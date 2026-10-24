import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('fishbot_description')

    xacro_file = os.path.join(pkg_share, 'urdf', 'fishbot', 'fishbot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'world', 'fishbot.sdf')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    model_name = 'fishbot'

    # ① 启动 Gazebo Harmonic
    #    注意：gz_sim.launch.py 没有 world 参数，世界是通过 gz_args 传的
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + world_file}.items())

    # ② URDF -> /robot_description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
        output='screen')

    # ③ 从 /robot_description 把机器人生成到世界里
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'default',
                   '-topic', 'robot_description',
                   '-name', model_name,
                   '-z', '0.3'],
        output='screen')

    # ④ 桥接：gz 话题 <-> ROS 话题
    #    方向符： [ = gz→ROS    ] = ROS→gz    @ = 双向
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 仿真时钟（Gazebo 出，ROS 收）
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # 差速驱动：速度指令 ROS→gz，里程计和 TF 是 gz→ROS
            f'/model/{model_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/model/{model_name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/model/{model_name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',

            # 传感器
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        remappings=[
            # 把 gz 那套名字映射成 ROS 里通用的名字
            (f'/model/{model_name}/cmd_vel',  '/cmd_vel'),
            (f'/model/{model_name}/odometry', '/odom'),
            (f'/model/{model_name}/tf',       '/tf'),
        ],
        parameters=[{'use_sim_time': True}],
        output='screen')

    # ⑤ RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen')

    # ⑥ SLAM 建图（slam_toolbox 异步模式），订阅 /scan + TF，发布 /map
    slam_params_file = os.path.join(pkg_share, 'config',
                                    'mapper_params_online_async.yaml')
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': slam_params_file,
        }.items(),
        condition=IfCondition(LaunchConfiguration('slam')),)

    # ⑦ 自动巡航建图（默认开启；想手动遥控就传 auto_explore:=false）
    auto_explore = Node(
        package='fishbot_description',
        executable='auto_explore',
        name='auto_explore',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('auto_explore')),
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'auto_explore', default_value='true',
            description='是否启动自动巡航避障节点（false 时可手动 teleop）'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='true=建图模式（起 slam_toolbox）；false=导航模式（用地图+AMCL）'),
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        bridge,
        slam,
        auto_explore,
        rviz,
    ])
