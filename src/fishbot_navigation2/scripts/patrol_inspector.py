#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""巡检（巡逻）小车：按航点自动巡航，到点拍照 + 语音播报 + 记录 CSV。

它做什么
--------
1. 等 Nav2 起来（bt_navigator / navigate_to_pose 就绪、AMCL 出了定位）；
2. 从 patrol_waypoints.yaml 读航点，一个点一个点地导航过去；
3. 每个点到了以后：停 pause_at_waypoint 秒 → 用相机拍一张 PNG → 用 espeak-ng 播报点位名；
4. 全程写一份 CSV 日志（谁、什么时候、到没到、花了多久、照片在哪）；
5. 走完一圈看 yaml 里的 loop：true 就再走一圈，false 就收工；
6. Ctrl+C 时取消当前导航目标 + 往 /cmd_vel 灌几帧零速度，保证小车停下来。

一般不用手动跑它，patrol_sim.launch.py 会一键拉起来：
    ros2 launch fishbot_navigation2 patrol_sim.launch.py

单独跑（Nav2 已经开着的时候）：
    ros2 run fishbot_navigation2 patrol_inspector
    ros2 run fishbot_navigation2 patrol_inspector --ros-args -p loop:=false
"""

from __future__ import annotations

import csv
import math
import os
import subprocess
import sys
import time
from datetime import datetime
from typing import TYPE_CHECKING, NotRequired, TypedDict, TypeVar

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from sensor_msgs.msg import Image


if TYPE_CHECKING:
    from numpy.typing import NDArray


class Waypoint(TypedDict):
    """航点配置：x、y 必填，name、yaw 可省略。"""

    x: float
    y: float
    name: NotRequired[str]
    yaw: NotRequired[float]


ImagePath = TypeVar(
    'ImagePath', str, os.PathLike[str],
)


# ======================================================================
#  一、几个纯函数（不碰 ROS，方便单独测试）
# ======================================================================

def yaw_to_quaternion(yaw: float) -> tuple[float, float]:
    """偏航角(弧度) → 四元数的 (z, w)。因为 roll=pitch=0，只用这两个分量。"""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def make_pose(
    frame_id: str, x: float, y: float, yaw: float, stamp: Time,
) -> PoseStamped:
    """拼一个 PoseStamped：Nav2 的目标点就是这种消息。"""
    pose = PoseStamped()
    pose.header.frame_id = frame_id          # 必须在 map 系里，Nav2 才知道你说的是哪儿
    pose.header.stamp = stamp
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0
    qz, qw = yaw_to_quaternion(float(yaw))
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def sanitize(name: str) -> str:
    """点位名 → 能当文件名用的字符串（中文保留，空格和斜杠换掉）。"""
    out = []
    for ch in str(name):
        out.append('_' if ch in ' /\\:*?"<>|' else ch)
    return ''.join(out).strip('_') or 'waypoint'


def image_msg_to_array(msg: Image) -> NDArray[np.uint8] | NDArray[np.float32]:
    """sensor_msgs/Image → numpy 数组。

    这里刻意不用 cv_bridge：它要额外装包，而且我们只要存图，
    用 numpy + PIL 两步就够了（PIL 和 numpy 都是系统自带的）。

    坑：msg.step 是"每行占多少字节"，可能比 宽×通道数 大（行对齐填充），
    所以先按 step 切成一行一行，再把多余的填充列去掉，最后才 reshape。
    """
    if msg.encoding in ('32FC1', '32FC1_depth', 'mono32f'):
        # 深度图：float32，单位米。先按 float32 切行。
        buf = np.frombuffer(msg.data, dtype=np.float32)
        rows = buf.reshape(msg.height, msg.step // 4)
        return rows[:, :msg.width]

    channels = {'mono8': 1, '8UC1': 1,
                'rgb8': 3, 'bgr8': 3, '8UC3': 3,
                'rgba8': 4, 'bgra8': 4, '8UC4': 4}.get(msg.encoding)
    if channels is None:
        raise ValueError('不认识的图像编码：%s' % msg.encoding)

    buf = np.frombuffer(msg.data, dtype=np.uint8)
    rows = buf.reshape(msg.height, msg.step)
    rows = rows[:, :msg.width * channels]
    if channels == 1:
        return rows.reshape(msg.height, msg.width)
    return rows.reshape(msg.height, msg.width, channels)


def save_image_msg(msg: Image, path: ImagePath) -> ImagePath:
    """把一帧 Image 存成 PNG 文件，返回 path。"""
    from PIL import Image as PILImage   # 放函数里 import：没装 PIL 也不影响巡逻本身

    arr = image_msg_to_array(msg)

    if arr.dtype != np.uint8:
        # 深度图之类的浮点数据：归一化到 0~255 纯为了"看得见"，不是原始深度值
        a = np.nan_to_num(arr, nan=0.0, posinf=0.0, neginf=0.0)
        hi = float(a.max())
        if hi > 0.0:
            a = a / hi * 255.0
        arr = a.astype(np.uint8)
    elif msg.encoding == 'bgr8':
        arr = arr[:, :, ::-1]                      # OpenCV 的 BGR 要翻成 RGB
    elif msg.encoding == 'bgra8':
        arr = arr[:, :, [2, 1, 0, 3]]

    PILImage.fromarray(arr).save(path)
    return path


# ======================================================================
#  二、巡检节点
# ======================================================================

class PatrolInspector(BasicNavigator):
    """继承 BasicNavigator：它已经帮我们建好了 NavigateToPose 的 action client，
    还有 goToPose() / isTaskComplete() / cancelTask() 这些现成方法。"""

    def __init__(self) -> None:
        super().__init__(node_name='patrol_inspector')

        # ---------------- 参数（launch 里会传，命令行也能覆盖） ----------------
        default_wp = os.path.join(get_package_share_directory('fishbot_navigation2'),
                                  'param', 'patrol_waypoints.yaml')
        self.declare_parameter('waypoints_file', default_wp)
        self.declare_parameter('log_dir', os.path.expanduser('~/patrol_log'))
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('voice_name', 'cmn')       # espeak-ng 的中文嗓音
        self.declare_parameter('goal_timeout', 120.0)     # 单个点位最多走多久(秒)
        self.declare_parameter('photo_timeout', 5.0)      # 等一帧新图像最多等多久(秒)
        self.declare_parameter('nav2_timeout', 180.0)     # 等 Nav2 起来最多等多久(秒)
        self.declare_parameter('arrive_tolerance', 0.6)   # 已经在目标这么近，就别再让 Nav2 挪了
        self.declare_parameter('loop', '')                # 留空=用 yaml 里的 loop；true/false 强制覆盖

        # ---------------- 读航点配置 ----------------
        wp_file = str(self.get_parameter('waypoints_file').value)
        if not os.path.isfile(wp_file):
            raise RuntimeError('找不到航点文件：%s' % wp_file)
        with open(wp_file, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f) or {}
        patrol = cfg.get('patrol', cfg)          # 兼容"有没有外面那层 patrol:"两种写法

        self.route: list[Waypoint] = patrol.get('waypoints') or []
        if not self.route:
            raise RuntimeError('航点文件里一个 waypoints 都没有：%s' % wp_file)
        self.frame_id: str = patrol.get('frame_id', 'map')
        self.loop = bool(patrol.get('loop', True))
        self.pause = float(patrol.get('pause_at_waypoint', 3.0))
        self.voice = bool(patrol.get('voice', True))
        self.photo = bool(patrol.get('photo', True))
        self.camera_topic = str(self.get_parameter('camera_topic').value)

        # 命令行强制覆盖 loop（留空则听 yaml 的）
        override = str(self.get_parameter('loop').value).strip().lower()
        if override in ('true', '1', 'yes', 'y'):
            self.loop = True
        elif override in ('false', '0', 'no', 'n'):
            self.loop = False

        self.goal_timeout = float(self.get_parameter('goal_timeout').value)
        self.photo_timeout = float(self.get_parameter('photo_timeout').value)
        self.voice_name = str(self.get_parameter('voice_name').value)
        self.arrive_tolerance = float(self.get_parameter('arrive_tolerance').value)

        # ---------------- 日志目录 / CSV ----------------
        self.log_dir = os.path.expanduser(str(self.get_parameter('log_dir').value))
        self.photo_dir = os.path.join(self.log_dir, 'photos')
        os.makedirs(self.photo_dir, exist_ok=True)
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_path = os.path.join(self.log_dir, 'patrol_%s.csv' % stamp)
        # utf-8-sig：带 BOM，双击用 Excel 打开中文不乱码
        self.csv_file = open(self.csv_path, 'w', newline='', encoding='utf-8-sig')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['序号', '时间', '点位名', '目标x', '目标y', '目标yaw',
                           '结果', '耗时(秒)', '照片文件'])
        self.csv_file.flush()

        # ---------------- 相机：只留最新一帧 ----------------
        self._img: Image | None = None
        self._img_seq = 0
        self._img_warned = False
        self._voice_warned = False
        # 传感器数据用 BEST_EFFORT + depth=1：只关心最新帧，丢帧无所谓
        self.create_subscription(
            Image, self.camera_topic, self._on_image,
            QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.BEST_EFFORT))

        # ---------------- 当前位置（AMCL 给的）：判断"是不是已经在点位旁边了" ----------------
        self._pose: tuple[float, float] | None = None
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl_pose,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=QoSReliabilityPolicy.RELIABLE))

        # ---------------- 安全停车用 ----------------
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    # ------------------------------------------------------------------
    #  相机 / 语音 小工具
    # ------------------------------------------------------------------
    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_image(self, msg: Image) -> None:
        self._img = msg
        self._img_seq += 1

    def speak(self, text: str) -> None:
        """调 espeak-ng 说话。用 Popen 不等待，所以不会卡住导航。"""
        if not self.voice:
            return
        try:
            subprocess.Popen(['espeak-ng', '-v', self.voice_name, '-s', '160', text],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            if not self._voice_warned:
                self._voice_warned = True
                self.get_logger().warn('没装 espeak-ng，语音播报关掉（sudo apt install espeak-ng）')

    def take_photo(self, index: int, name: str) -> str:
        """等到一帧"新"图像并存盘，返回文件路径；拿不到就返回空字符串。"""
        seq_before = self._img_seq
        deadline = time.time() + self.photo_timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)     # 收图靠 spin
            if self._img_seq > seq_before and self._img is not None:
                break

        if self._img is None:
            if not self._img_warned:
                self._img_warned = True
                self.get_logger().warn(
                    '拿不到相机图像（订阅的是 %s）。三步排查：\n'
                    '  ① gz topic -l | grep camera   → 看 Gazebo 到底发不发\n'
                    '  ② ros2 topic hz %s            → 看桥接通没通\n'
                    '  ③ 桥接写在 patrol_sim.launch.py 的 camera_bridge 那段'
                    % (self.camera_topic, self.camera_topic))
            return ''

        fname = '%02d_%s_%s.png' % (index, sanitize(name),
                                    datetime.now().strftime('%H%M%S'))
        path = os.path.join(self.photo_dir, fname)
        try:
            save_image_msg(self._img, path)
            return path
        except Exception as exc:                       # 存图失败不该让巡逻中断
            self.get_logger().error('存图失败：%s: %s' % (type(exc).__name__, exc))
            return ''

    def stop_robot(self) -> None:
        """连发几帧零速度，确保底盘真的停住。"""
        zero = Twist()
        for _ in range(10):
            self._cmd_pub.publish(zero)
            time.sleep(0.05)

    def rest(self, seconds: float) -> None:
        """停留 N 秒，但期间继续 spin（不然 /amcl_pose、TF 会断掉）。"""
        end = time.time() + float(seconds)
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

    # ------------------------------------------------------------------
    #  等 Nav2 / 走一个点位
    # ------------------------------------------------------------------
    def wait_for_nav2(self) -> None:
        """分两步等：先等 NavigateToPose 动作服务器，再等 AMCL 出定位。

        为什么不用官方的 waitUntilNav2Active()？
        它内部是 while 死循环等生命周期节点变 active，一旦某个节点没起来，
        它会**静默卡死**（日志只在 debug 级别）。这里改成带超时的等待，
        卡住了会明确报错，让你知道该去看谁的日志。
        """
        timeout = float(self.get_parameter('nav2_timeout').value)
        self.get_logger().info('等 Nav2 起来（最多 %.0f 秒）…' % timeout)

        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.nav_to_pose_client.server_is_ready():
                break
            rclpy.spin_once(self, timeout_sec=0.2)
        else:
            raise RuntimeError(
                '等了 %.0fs 还没等到 navigate_to_pose 动作服务器，Nav2 没起来。' % timeout)
        self.get_logger().info('navigate_to_pose 已就绪（等了 %.1fs）' % (time.time() - t0))

        # ★ 光看"动作服务器在"是不够的！
        #   bt_navigator 在 configure 之后就会建好 action server，但它可能一直停在
        #   inactive —— 例如 map_server 加载地图失败会让整个导航组 bringup 中止。
        #   这时送目标会被直接拒绝：
        #       [bt_navigator]: Action server is inactive. Rejecting the goal.
        #   而 BasicNavigator 只会记一条 error 就返回，结果变成 UNKNOWN，
        #   脚本就会一圈一圈空转（实测空转了 672 圈）。
        #   所以这里再等它真正变成 active，等不到就带着排查线索报错退出。
        state_cli = self.create_client(GetState, 'bt_navigator/get_state')
        t0 = time.time()
        state = '未知'
        while time.time() - t0 < timeout:
            if state_cli.wait_for_service(timeout_sec=1.0):
                fut = state_cli.call_async(GetState.Request())
                rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
                if fut.result() is not None:
                    state = fut.result().current_state.label
                    if state == 'active':
                        break
            rclpy.spin_once(self, timeout_sec=0.2)

        if state != 'active':
            raise RuntimeError(
                'bt_navigator 一直不是 active（当前状态：%s，等了 %.0fs）。\n'
                '    最常见原因：map_server 没能加载地图，导致整个导航组 bringup 中止，'
                '于是没有 map→odom 的 TF，planner_server 激活超时。排查三步：\n'
                '      ① ros2 lifecycle get /map_server\n'
                '      ② ls -l install/fishbot_navigation2/share/fishbot_navigation2/maps/\n'
                '         —— install(DIRECTORY) 是编译时"快照"文件列表的：新生成的地图\n'
                '            （map_saver_cli 或 make_map_from_world 产出的 room.pgm）\n'
                '            不重新 colcon build 就不会被装进 install/，\n'
                '            map_server 会报 Failed to load image file ... room.pgm\n'
                '      ③ 看终端里 map_server 的报错' % (state, timeout))
        self.get_logger().info('bt_navigator 已 active，导航栈可用')

        t0 = time.time()
        while time.time() - t0 < 60.0 and self._pose is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._pose is not None:
            self.get_logger().info('AMCL 已发布 /amcl_pose，定位就绪（当前 %.2f, %.2f）'
                                   % self._pose)
        else:
            self.get_logger().warn('一直没收到 /amcl_pose：检查 AMCL 是否 active、'
                                   'nav2_params.yaml 里 amcl.set_initial_pose 是否为 true')

        # 第一次规划前把代价地图清一下，避免开机残留的障碍把起点围死
        try:
            self.clearAllCostmaps()
        except Exception as exc:
            self.get_logger().warn('清代价地图失败（不影响继续）：%s' % exc)

    def visit_waypoint(self, index: int, wp: Waypoint) -> TaskResult:
        name = str(wp.get('name', '点位%d' % index))
        x, y = float(wp['x']), float(wp['y'])
        yaw = float(wp.get('yaw', 0.0))

        self.get_logger().info('─' * 56)
        self.get_logger().info('[%d/%d] 出发 → %s  (x=%.2f, y=%.2f, yaw=%.2f)'
                               % (index, len(self.route), name, x, y, yaw))

        # 已经在点位旁边了就别再让 Nav2 挪：距离小于 arrive_tolerance 时，
        # 送目标反而会触发 "Failed to make progress"（原地磨几十秒）。
        # 开机时 1 号点位离出生点很近的话，这一段能省掉一大截无谓等待。
        if self._pose is not None:
            dx, dy = self._pose[0] - x, self._pose[1] - y
            near = math.hypot(dx, dy)
            if near <= self.arrive_tolerance:
                self.get_logger().info('    已经在 %.2fm 以内（容差 %.2fm），直接巡检，不再导航'
                                       % (near, self.arrive_tolerance))
                self.speak('已在%s' % name)
                photo = self.take_photo(index, name) if self.photo else ''
                if photo:
                    self.get_logger().info('    照片已存：%s' % photo)
                self.rest(self.pause)
                self.write_csv(index, name, x, y, yaw, 'ALREADY_HERE', 0.0, photo)
                return TaskResult.SUCCEEDED

        self.speak('前往%s' % name)

        goal = make_pose(self.frame_id, x, y, yaw, self.get_clock().now().to_msg())
        t0 = time.time()
        self.goToPose(goal)

        timed_out = False
        last_log = 0.0
        while not self.isTaskComplete():
            fb = self.getFeedback()
            now = time.time()
            if fb is not None and now - last_log > 3.0:
                last_log = now
                dist = getattr(fb, 'distance_remaining', float('nan'))
                self.get_logger().info('    导航中… 还剩 %.2f m' % dist)
            if now - t0 > self.goal_timeout:
                self.get_logger().warn('    超过 %.0fs 还没到，放弃这个点位'
                                       % self.goal_timeout)
                self.cancelTask()
                timed_out = True
                break
            time.sleep(0.05)          # isTaskComplete() 内部已经在 spin 了

        result = TaskResult.CANCELED if timed_out else self.getResult()
        cost = time.time() - t0
        photo = ''

        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('    到达 %s（用时 %.1fs）' % (name, cost))
            self.speak('已到达%s' % name)
            if self.photo:
                photo = self.take_photo(index, name)
                if photo:
                    self.get_logger().info('    照片已存：%s' % photo)
            self.rest(self.pause)
        else:
            self.get_logger().warn('    %s 没走到（%s），跳下一个' % (name, result.name))
            self.speak('%s未能到达' % name)

        self.write_csv(index, name, x, y, yaw, result.name, cost, photo)
        return result

    def write_csv(
        self, index: int, name: str, x: float, y: float, yaw: float,
        result: str, cost: float, photo: str,
    ) -> None:
        self.csv.writerow([index, datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                           name, '%.3f' % x, '%.3f' % y, '%.3f' % yaw,
                           result, '%.1f' % cost, photo])
        self.csv_file.flush()          # 立刻落盘：就算 Ctrl+C 也不会丢记录

    # ------------------------------------------------------------------
    #  主流程
    # ------------------------------------------------------------------
    def run(self) -> None:
        self.get_logger().info('巡检路线（来自 %s）：'
                               % self.get_parameter('waypoints_file').value)
        for i, wp in enumerate(self.route, start=1):
            self.get_logger().info('    %d. %s  (%.2f, %.2f)'
                                   % (i, wp.get('name', ''), wp['x'], wp['y']))
        self.get_logger().info('循环=%s，每点停留 %.1fs，拍照=%s，语音=%s，相机=%s'
                               % (self.loop, self.pause, self.photo, self.voice,
                                  self.camera_topic))
        self.get_logger().info('CSV 日志：%s' % self.csv_path)

        self.wait_for_nav2()

        lap = 0
        while rclpy.ok():
            lap += 1
            self.get_logger().info('=' * 56)
            self.get_logger().info('第 %d 圈开始' % lap)
            rejected = 0
            for i, wp in enumerate(self.route, start=1):
                result = self.visit_waypoint(i, wp)
                # ★ 保险丝：目标被连续拒绝说明导航栈根本没在工作，
                #   这时候绝不能一圈一圈空转（实测会刷 600+ 圈日志）
                rejected = rejected + 1 if result == TaskResult.UNKNOWN else 0
                if rejected >= 3:
                    raise RuntimeError(
                        '连续 3 个点位都被 Nav2 拒绝（结果 UNKNOWN），导航栈没在正常工作。\n'
                        '    先别空转，查这个：ros2 lifecycle get /bt_navigator')
            self.get_logger().info('第 %d 圈结束' % lap)
            if not self.loop:
                self.get_logger().info('loop=false，一圈走完收工')
                break
        self.get_logger().info('巡检结束')

    def shutdown(self) -> None:
        """收尾：取消目标 → 停车 → 关 CSV → 销毁节点。"""
        try:
            self.cancelTask()
        except Exception:
            pass
        self.stop_robot()
        try:
            self.csv_file.close()
        except Exception:
            pass
        print('CSV 日志已保存：%s' % self.csv_path)
        try:
            self.destroyNode()
        except Exception:
            pass


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node: PatrolInspector | None = None
    try:
        node = PatrolInspector()
        node.run()
    except KeyboardInterrupt:
        print('\n收到 Ctrl+C，取消导航目标并停车…')
    except Exception as exc:
        print('巡检节点出错：%s: %s' % (type(exc).__name__, exc), file=sys.stderr)
    finally:
        if node is not None:
            node.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
