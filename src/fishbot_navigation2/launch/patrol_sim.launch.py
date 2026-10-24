"""一键启动"巡检小车"：Gazebo（导航模式）+ Nav2 + 航点巡检任务。

    ros2 launch fishbot_navigation2 patrol_sim.launch.py

它其实就把三件事按顺序拼起来：
    ① gazebo_sim.launch.py   且 slam:=false  auto_explore:=false → 起仿真，用现成地图不建图
    ② navigation2.launch.py  → 起 Nav2（AMCL 定位 + 规划 + 控制），读 maps/room.yaml
    ③ patrol_inspector       → 我们自己的巡检脚本，按 param/patrol_waypoints.yaml 跑点位

顺手还多干了两件事：
    · 单独起一个 ros_gz_bridge，把 Gazebo 的相机桥到 ROS（拍照要用）
    · 巡检脚本延后 10 秒启动，等 Nav2 的生命周期节点变 active，日志干净一些

只想起仿真不巡检：
    ros2 launch fishbot_navigation2 patrol_sim.launch.py patrol:=false
想换地图 / 参数 / 航点，改对应的 launch 参数（见下面 DeclareLaunchArgument）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg = get_package_share_directory('fishbot_description')
    nav_pkg = get_package_share_directory('fishbot_navigation2')

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    waypoints_file = LaunchConfiguration('waypoints_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_topic = LaunchConfiguration('camera_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    patrol = LaunchConfiguration('patrol')

    # ① 仿真：导航模式 = 不建图、不自动探索
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_pkg, 'launch', 'gazebo_sim.launch.py')),
        launch_arguments={
            'slam': 'false',          # ★ 用现成地图 + AMCL，不起 slam_toolbox
            'auto_explore': 'false',  # ★ 不跑 frontier 探索，改由 Nav2 按航点走
        }.items())

    # ② Nav2：AMCL 定位 + 全局/局部代价地图 + 规划控制
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg, 'launch', 'navigation2.launch.py')),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items())

    # ③ 相机桥接。Gazebo 这边实际有这么几个话题（都是实测确认的，别照抄教程）：
    #      /camera_rgb    gz.msgs.Image            彩色图 ← 普通 camera 传感器，拍照用
    #      /camera        gz.msgs.Image            深度图（ROS 侧 32FC1！）
    #      /camera_info   gz.msgs.CameraInfo       深度相机内参
    #      /camera/points gz.msgs.PointCloudPacked 点云（本脚本用不到）
    #    踩过的坑：depth_camera 传感器的 <topic> 上发的是深度图，不是彩色图，
    #    而且 camera_info 叫 /camera_info（不是 /camera/camera_info）。
    #    注意 ros_gz_bridge 一条规则里 ROS 名和 GZ 名是同一个字符串，
    #    想让 ROS 侧叫别的名字只能用 remappings 改（下面就是这么做的）。
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/camera_rgb@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/camera_rgb', camera_topic),        # 彩色 → /camera/image_raw
            ('/camera', depth_topic),             # 深度 → /camera/depth
            ('/camera_info', camera_info_topic),
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    # ④ 巡检脚本：延后 10 秒，等 Nav2 起来
    inspector = Node(
        package='fishbot_navigation2',
        executable='patrol_inspector',
        name='patrol_inspector',
        parameters=[{
            'use_sim_time': use_sim_time,
            'waypoints_file': waypoints_file,
            'camera_topic': camera_topic,
        }],
        condition=IfCondition(patrol),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(nav_pkg, 'maps', 'room.yaml'),
            description='地图 yaml（默认用世界文件渲染的真值地图）'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav_pkg, 'param', 'nav2_params.yaml'),
            description='Nav2 参数文件（已按 base_footprint / fishbot/odom 改好）'),
        DeclareLaunchArgument(
            'waypoints_file',
            default_value=os.path.join(nav_pkg, 'param', 'patrol_waypoints.yaml'),
            description='巡检航点文件（换路线只改这个 yaml）'),
        DeclareLaunchArgument(
            'camera_topic', default_value='/camera/image_raw',
            description='彩色相机在 ROS 侧的名字（巡检拍照订阅的就是它）'),
        DeclareLaunchArgument(
            'depth_topic', default_value='/camera/depth',
            description='深度相机在 ROS 侧的名字（32FC1，加分项用）'),
        DeclareLaunchArgument(
            'camera_info_topic', default_value='/camera/camera_info',
            description='深度相机内参在 ROS 侧的名字'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true', description='用仿真时钟'),
        DeclareLaunchArgument(
            'patrol', default_value='true',
            description='false = 只起仿真和 Nav2，不跑巡检任务'),

        gazebo,
        nav2,
        camera_bridge,
        TimerAction(period=10.0, actions=[inspector]),
    ])
