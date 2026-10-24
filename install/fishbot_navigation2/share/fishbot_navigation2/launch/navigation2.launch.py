"""Nav2 导航启动：用现成地图 + AMCL（不启 SLAM）。

用法：
    ros2 launch fishbot_navigation2 navigation2.launch.py
    ros2 launch fishbot_navigation2 navigation2.launch.py map:=/别的/路径/room.yaml
    ros2 launch fishbot_navigation2 navigation2.launch.py --show-args     # 只看参数不启动

三个默认值都指向本项目里的实际文件：
    params_file → param/nav2_params.yaml   （已按 fishbot 的 base_footprint / fishbot/odom 改好）
    map         → maps/room.yaml           （由世界文件渲染的真值地图）
    use_sim_time→ true                     （仿真里必须）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav2_bringup = get_package_share_directory('nav2_bringup')
    my_pkg = get_package_share_directory('fishbot_navigation2')

    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(my_pkg, 'param', 'nav2_params.yaml'),
            description='Nav2 参数文件（已按 fishbot 的坐标系改过）'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(my_pkg, 'maps', 'room.yaml'),
            description='地图（由世界文件渲染，所有房间连通）'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='用仿真时钟'),

        # 直接用官方 bringup，把几个关键开关传进去
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'slam': 'False',             # ★ 用现成地图 + AMCL；SLAM 由 gazebo_sim 那边控制
                'map': map_file,
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': 'true',         # 自动激活生命周期节点，不用手动 configure
                'use_composition': 'False',  # 仿真里不开组合：日志好读、好排错
            }.items(),
        ),
    ])
