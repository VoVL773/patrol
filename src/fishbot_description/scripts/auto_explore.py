#!/usr/bin/env python3
"""自主探索建图：基于"边界（frontier）"的探索节点。

为什么不再用旧的反应式逻辑
    旧版见同目录 `auto_explore_reactive.py.bak`：前方有障碍就转向"更宽的一侧"。
    在有房间的户型里，房间是封闭小空间、大厅是开阔大空间 —— 它每次都选大厅，
    结构性回避门洞，所以永远进不去房间，地图只有客厅。

新版的思路（先看图，再决定去哪）
    1) 订阅 /map（slam_toolbox 发布），找出"已知空闲格 紧邻 未知格"的边界点（frontier）
    2) 把相邻的边界点**聚成一簇**（避免贴着墙一个个啃）
    3) 挑"离自己最近 + 有视线（中间不隔墙）+ 没被拉黑"的簇，朝**簇心**走
    4) 朝目标：先转向对准，再直行；同时用 /scan 做紧急避障
    5) 到达 / 超时 / 卡住 → 拉黑，换下一簇
    6) 再也找不到 frontier → 宣布探索完成，并打印存图命令

为什么这样才进得去房间
    门洞在地图上就是"空闲走廊连着未知区域"的边界。frontier 算法主动朝"未知"走，
    并且**有视线检查**，所以不会因为房间比大厅窄就掉头，也不会去啃墙后面的点。

依赖：/map、/scan、/odom、TF(map → base_footprint)
"""
from __future__ import annotations

import math
import time
from collections.abc import Sequence


import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

Point = tuple[float, float]
Frontier = tuple[float, float, int]


# ===== 调参区 =====
CONTROL_HZ = 10.0            # 控制频率
REPLAN_PERIOD = 1.0          # 每隔多久重新挑一次目标（秒）
FORWARD_SPEED = 0.18         # m/s，慢一点，建图质量好
BACK_SPEED = -0.12           # m/s
YAW_KP = 1.8                 # 朝向比例增益
MAX_TURN = 1.0               # rad/s 上限
ALIGN_TOL = 0.30             # 朝向误差小于它才允许直行（rad）
FRONT_STOP = 0.38            # 正前方这个距离内有障碍就转向让开（m）
DANGER_STOP = 0.22           # 更近就先后退（m）
FRONT_HALF_ANGLE = 25.0      # 紧急避障用的前方扇区半角（度）
SIDE_ANGLE = (40.0, 140.0)   # 左右侧扇区（度，左侧为正）
GOAL_TOL = 0.40              # 到目标这么近算到达（m）
MIN_GOAL_DIST = 1.0          # 比这更近的边界簇忽略（避免贴着墙一点点啃）
MAX_GOAL_DIST = 8.0          # 比这更远的忽略（不横跨整个房子）
CLUSTER_RADIUS = 0.5         # 边界点聚类半径（m）
BLACKLIST_RADIUS = 0.80      # 拉黑半径（m）
LOS_STEP = 0.05              # 视线检查采样步长（m）
GOAL_TIMEOUT = 30.0          # 单个目标最长尝试时间（秒）
STUCK_TIME = 3.0             # 判定"轮子转但车不动"的时间（秒）
STUCK_SPEED = 0.05           # 实测速度低于它算没动（m/s）
BACK_TIME = 0.6              # 后退持续（秒）
MAX_RUNTIME = 900.0          # 总时限（秒），到点强制停
EMPTY_FRONTIER_ROUNDS = 15   # 连续这么多轮找不到边界就宣布探索完成


# ===== 纯函数区（不依赖 ROS，便于单独测试）=====
def find_frontiers(
    data: Sequence[int], width: int, height: int,
    origin_x: float, origin_y: float, resolution: float,
    stride: int = 2, min_unknown_neighbors: int = 1,
) -> list[Frontier]:
    """找出所有「空闲格且紧邻未知格」的格子。

    data: OccupancyGrid.data（0=空闲, 100=障碍, -1=未知）
    返回 [(x, y, 未知邻居个数), ...]，世界坐标。
    """
    out: list[Frontier] = []
    for r in range(1, height - 1, stride):
        base = r * width
        for c in range(1, width - 1, stride):
            i = base + c
            if data[i] != 0:                      # 只从空闲格出发
                continue
            unknown = 0
            if data[i + 1] == -1:
                unknown += 1
            if data[i - 1] == -1:
                unknown += 1
            if data[i + width] == -1:
                unknown += 1
            if data[i - width] == -1:
                unknown += 1
            if unknown >= min_unknown_neighbors:
                out.append((origin_x + (c + 0.5) * resolution,
                            origin_y + (r + 0.5) * resolution,
                            unknown))
    return out


def has_line_of_sight(
    data: Sequence[int], width: int, height: int,
    origin_x: float, origin_y: float, resolution: float,
    x0: float, y0: float, x1: float, y1: float, step: float = LOS_STEP,
) -> bool:
    """从 (x0,y0) 到 (x1,y1) 的直线是否不穿过障碍格（用来排除"墙后面的目标"）。"""
    dist = math.hypot(x1 - x0, y1 - y0)
    n = max(1, int(dist / step))
    for k in range(1, n):
        x = x0 + (x1 - x0) * k / n
        y = y0 + (y1 - y0) * k / n
        c = int((x - origin_x) / resolution)
        r = int((y - origin_y) / resolution)
        if not (0 <= c < width and 0 <= r < height):
            return False
        if data[r * width + c] >= 50:             # 撞到障碍
            return False
    return True


def cluster_frontiers(
    frontiers: Sequence[Frontier], radius: float = CLUSTER_RADIUS,
) -> list[list[Frontier]]:
    """把相邻的边界点聚成簇（贪心），返回 [[(x,y,unknown), ...], ...]。"""
    clusters: list[list[Frontier]] = []
    for p in frontiers:
        for cl in clusters:
            if any(abs(p[0] - q[0]) <= radius and abs(p[1] - q[1]) <= radius for q in cl):
                cl.append(p)
                break
        else:
            clusters.append([p])
    return clusters


def pick_goal(
    frontiers: Sequence[Frontier], robot_x: float, robot_y: float,
    blacklist: Sequence[Point], data: Sequence[int], width: int, height: int,
    origin_x: float, origin_y: float, resolution: float,
    min_dist: float = MIN_GOAL_DIST, max_dist: float = MAX_GOAL_DIST,
    blacklist_radius: float = BLACKLIST_RADIUS, require_los: bool = True,
) -> Point | None:
    """挑"最近的、有视线、没被拉黑"的边界簇。

    返回该簇的代表点：有视线时给**簇心**（朝未知区域里推），否则退化给簇内最近点。
    没有合适目标时返回 None。
    """
    candidates = []
    for cluster in cluster_frontiers(frontiers):
        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)
        nearest = min(cluster, key=lambda p: math.hypot(p[0] - robot_x, p[1] - robot_y))
        d = math.hypot(nearest[0] - robot_x, nearest[1] - robot_y)
        if d < min_dist or d > max_dist:
            continue
        if any(math.hypot(nearest[0] - bx, nearest[1] - by) < blacklist_radius
               for bx, by in blacklist):
            continue
        los = has_line_of_sight(data, width, height, origin_x, origin_y, resolution,
                                robot_x, robot_y, nearest[0], nearest[1])
        candidates.append((d, los, cx, cy, nearest))

    if not candidates:
        return None
    if require_los:
        visible = [c for c in candidates if c[1]]
        if visible:
            candidates = visible
    candidates.sort(key=lambda c: c[0])
    d, los, cx, cy, nearest = candidates[0]
    # ★ 返回簇内"离机器人最近"的边界点，而不是簇心：
    #   簇心可能落在墙后面（例如房间深处），直线开过去会撞墙；
    #   而边界点一定在"已知空闲区"，配合上面的视线检查，目标就总是可达的。
    return (nearest[0], nearest[1])


def normalize_angle(a: float) -> float:
    """把角度折到 [-pi, pi]，避免跨 ±π 时绕远路掉头。"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class FrontierExplorer(Node):

    def __init__(self) -> None:
        super().__init__('auto_explore')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, 1)
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.create_subscription(Odometry, 'odom', self.on_odom, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.grid: OccupancyGrid | None = None
        self.ranges: Sequence[float] | None = None
        self.angle_min = 0.0
        self.angle_inc = 0.0
        self.actual_vx = 0.0
        self.pose: tuple[float, float, float] | None = None  # map 系下 (x, y, yaw)

        self.goal: Point | None = None
        self.goal_since = 0.0
        self.blacklist: list[Point] = []          # 试过的目标点（世界坐标）
        self.empty_rounds = 0
        self.stuck_since: float | None = None
        self.back_until = 0.0
        self.turn_dir = 1
        self.start_time = time.monotonic()
        self.done = False

        self.create_timer(1.0 / CONTROL_HZ, self.control_loop)
        self.create_timer(REPLAN_PERIOD, self.replan)
        self.get_logger().info('auto_explore（frontier 版）已启动：先看图，再朝"未知区域"走')

    # ---------- 回调 ----------
    def on_map(self, msg: OccupancyGrid) -> None:
        self.grid = msg

    def on_scan(self, msg: LaserScan) -> None:
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_inc = msg.angle_increment

    def on_odom(self, msg: Odometry) -> None:
        self.actual_vx = msg.twist.twist.linear.x

    # ---------- 工具 ----------
    def _idx_at(self, deg: float) -> int:
        a = math.radians(deg) % (2.0 * math.pi)
        return int((a - self.angle_min) / self.angle_inc) % len(self.ranges)

    def _sector_min(self, lo_deg: float, hi_deg: float) -> float:
        """取 [lo, hi] 扇区内的最近距离（度，逆时针为正，支持跨 0°）。"""
        best = float('inf')
        i = self._idx_at(lo_deg)
        end = self._idx_at(hi_deg)
        n = len(self.ranges)
        steps = 0
        while True:
            r = self.ranges[i]
            if r == r and r != float('inf') and r > 0.0:
                best = min(best, r)
            if i == end or steps > n:
                break
            i = (i + 1) % n
            steps += 1
        return best

    def _update_pose(self) -> bool:
        """从 TF 取 map → base_footprint 的位姿（SLAM 会维护这条变换）。"""
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except Exception:                       # noqa: BLE001
            return False
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (t.x, t.y, yaw)
        return True

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def finish(self, why: str) -> None:
        self.done = True
        self._stop()
        self.get_logger().info('=' * 56)
        self.get_logger().info(why)
        self.get_logger().info('下一步——存图：')
        self.get_logger().info('  ros2 run nav2_map_server map_saver_cli -f <你的路径>/maps/room')
        self.get_logger().info('再体检：')
        self.get_logger().info('  python3 ~/dsh工作区/tools/地图覆盖检查.py')
        self.get_logger().info('=' * 56)

    # ---------- 选目标 ----------
    def replan(self) -> None:
        if self.done or self.grid is None or not self._update_pose():
            return
        info = self.grid.info
        frontiers = find_frontiers(self.grid.data, info.width, info.height,
                                   info.origin.position.x, info.origin.position.y,
                                   info.resolution)
        rx, ry, _ = self.pose

        # 目标还新鲜就继续用（滞回：避免在两个目标之间左右横跳）
        if self.goal is not None:
            age = time.monotonic() - self.goal_since
            dist = math.hypot(self.goal[0] - rx, self.goal[1] - ry)
            if age < GOAL_TIMEOUT and dist > GOAL_TOL:
                return

        goal = pick_goal(frontiers, rx, ry, self.blacklist,
                         self.grid.data, info.width, info.height,
                         info.origin.position.x, info.origin.position.y,
                         info.resolution)
        if goal is None:
            self.goal = None
            self.empty_rounds += 1
            if self.empty_rounds >= EMPTY_FRONTIER_ROUNDS:
                self.finish('地图里已经没有新的边界了：探索完成 ✅')
            return

        self.empty_rounds = 0
        self.goal = goal
        self.goal_since = time.monotonic()
        dist = math.hypot(goal[0] - rx, goal[1] - ry)
        self.get_logger().info(
            f'新目标 ({goal[0]:.2f}, {goal[1]:.2f})  距离 {dist:.2f} m  '
            f'(边界点 {len(frontiers)} 个，已拉黑 {len(self.blacklist)} 个)')

    # ---------- 主控制 ----------
    def control_loop(self) -> None:
        cmd = Twist()
        if self.done:
            self.cmd_pub.publish(cmd)
            return
        now = time.monotonic()
        if now - self.start_time > MAX_RUNTIME:
            self.finish(f'到达总时限 {MAX_RUNTIME:.0f} s，停止探索（可用 teleop 补扫剩下的角落）')
            return
        if self.ranges is None or not self._update_pose():
            self.cmd_pub.publish(cmd)          # 等 /scan 和 TF
            return

        # ① 后退让开
        if now < self.back_until:
            cmd.linear.x = BACK_SPEED
            self.cmd_pub.publish(cmd)
            return

        # ② 紧急避障优先于去目标
        front = self._sector_min(-FRONT_HALF_ANGLE, FRONT_HALF_ANGLE)
        if front < DANGER_STOP:
            self.get_logger().warn(f'贴太近 {front:.2f} m，先后退')
            self.back_until = now + BACK_TIME
            cmd.linear.x = BACK_SPEED
            self.cmd_pub.publish(cmd)
            return
        if front < FRONT_STOP:
            left = self._sector_min(*SIDE_ANGLE)
            right = self._sector_min(-SIDE_ANGLE[1], -SIDE_ANGLE[0])
            self.turn_dir = 1 if left >= right else -1
            cmd.angular.z = MAX_TURN * self.turn_dir
            self.cmd_pub.publish(cmd)
            return

        # ③ 朝目标走
        if self.goal is None:
            self.cmd_pub.publish(cmd)
            return
        rx, ry, yaw = self.pose
        err = normalize_angle(math.atan2(self.goal[1] - ry, self.goal[0] - rx) - yaw)
        cmd.angular.z = max(-MAX_TURN, min(MAX_TURN, YAW_KP * err))
        if abs(err) < ALIGN_TOL:
            cmd.linear.x = FORWARD_SPEED
        self.cmd_pub.publish(cmd)

        # ④ 到达判定
        if math.hypot(self.goal[0] - rx, self.goal[1] - ry) < GOAL_TOL:
            self.get_logger().info('到达目标 → 拉黑它，换下一簇')
            self.blacklist.append(self.goal)
            self.goal = None
            return

        # ⑤ 卡住判定（轮子转但车没动）
        if cmd.linear.x > 0.05 and abs(self.actual_vx) < STUCK_SPEED:
            if self.stuck_since is None:
                self.stuck_since = now
            elif now - self.stuck_since > STUCK_TIME:
                self.get_logger().warn('轮子在转但车没动 → 拉黑当前目标，后退换方向')
                self.blacklist.append(self.goal)
                self.goal = None
                self.stuck_since = None
                self.back_until = now + BACK_TIME
        else:
            self.stuck_since = None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()                      # 退出前发零速，防止车继续跑
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
