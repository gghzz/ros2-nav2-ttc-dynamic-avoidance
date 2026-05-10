import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 获取与拼接默认路径
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    rviz_config_dir = os.path.join(
        nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'
    )

    # 创建 Launch 配置
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time', default='true'
    )

    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map',
        default=os.path.join(fishbot_navigation2_dir, 'maps', 'map.yaml')
    )

    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file',
        default=os.path.join(
            fishbot_navigation2_dir,
            'config',
            'nav2_params.yaml'
        )
    )

    return launch.LaunchDescription([
        # 声明 Launch 参数
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation Gazebo clock if true'
        ),

        launch.actions.DeclareLaunchArgument(
            'map',
            default_value=map_yaml_path,
            description='Full path to map file to load'
        ),

        launch.actions.DeclareLaunchArgument(
            'params_file',
            default_value=nav2_param_path,
            description='Full path to param file to load'
        ),

        # 启动 Nav2，但是把 Nav2 内部所有 cmd_vel 输出重映射成 cmd_vel_raw
        #
        # 目的：
        # controller_server / behavior_server
        #       ↓
        # /cmd_vel_raw
        #
        # 这样原始速度不会直接给 Gazebo 小车执行
        launch.actions.GroupAction([
            launch_ros.actions.SetRemap(
                src='cmd_vel',
                dst='cmd_vel_raw'
            ),

            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_bringup_dir, '/launch', '/bringup_launch.py']
                ),
                launch_arguments={
                    'map': map_yaml_path,
                    'use_sim_time': use_sim_time,
                    'params_file': nav2_param_path
                }.items(),
            ),
        ]),

        # 单独启动 collision_monitor
        #
        # 它会读取 nav2_params.yaml 里面的 collision_monitor 参数
        launch_ros.actions.Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[nav2_param_path],
        ),

        # 单独给 collision_monitor 配一个 lifecycle manager
        #
        # 否则 collision_monitor 可能只是启动了，但没有进入 active 状态
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_collision_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['collision_monitor']
            }]
        ),

        # 启动 RViz
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])