import os

import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # ============================================================
    # 获取包路径
    # ============================================================
    fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
    fishbot_description_dir = get_package_share_directory('fishbot_description')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    rviz_config_dir = os.path.join(
        nav2_bringup_dir,
        'rviz',
        'nav2_default_view.rviz'
    )

    # ============================================================
    # Launch 参数
    # ============================================================
    use_sim_time = launch.substitutions.LaunchConfiguration(
        'use_sim_time',
        default='true'
    )

    map_yaml_path = launch.substitutions.LaunchConfiguration(
        'map',
        default=os.path.join(
            fishbot_navigation2_dir,
            'maps',
            'map.yaml'
        )
    )

    nav2_param_path = launch.substitutions.LaunchConfiguration(
        'params_file',
        default=os.path.join(
            fishbot_navigation2_dir,
            'config',
            'nav2_params.yaml'
        )
    )

    world_file_path = launch.substitutions.LaunchConfiguration(
        'world_file_path',
        default=os.path.join(
            fishbot_description_dir,
            'world',
            'room.world'
        )
    )

    moving_model_name = launch.substitutions.LaunchConfiguration(
        'moving_model_name',
        default=''
    )

    # ============================================================
    # 1. 启动 Nav2
    #
    # 这里把 Nav2 内部 cmd_vel 重映射成 cmd_vel_raw
    #
    # 正常链路：
    # controller_server / behavior_server
    #       ↓
    # /cmd_vel_raw
    #       ↓
    # velocity_smoother
    #       ↓
    # /cmd_vel
    #
    # 注意：
    # intersection_speed_gate.py 和 collision_monitor
    # 不要放进这个 GroupAction 里面，
    # 否则它们的 cmd_vel 也可能被重映射。
    # ============================================================
    nav2_group = launch.actions.GroupAction([
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
    ])

    # ============================================================
    # 2. dynamic_scan_filter.py
    #
    # 功能：
    # /scan + /map + TF
    #       ↓
    # 过滤静态墙体 / 静态障碍物
    #       ↓
    # 发布 /dynamic_scan
    # ============================================================
    dynamic_scan_filter_node = launch_ros.actions.Node(
        package='fishbot_navigation2',
        executable='dynamic_scan_filter.py',
        name='dynamic_scan_filter',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,

            # 输入
            'scan_topic': '/scan',
            'map_topic': '/map',

            # 输出
            'dynamic_scan_topic': '/dynamic_scan',

            # 坐标系
            'map_frame': 'map',

            # 静态地图过滤参数
            'occupied_thresh': 65,
            'static_margin_cells': 2,
        }]
    )

    # ============================================================
    # 3. intersection_supervisor.py
    #
    # 功能：
    # 订阅 /dynamic_scan
    # 判断动态障碍物距离、运动趋势、TTC 碰撞风险
    # 发布 /intersection_decision
    #
    # 典型输出：
    # NORMAL_NAVIGATION
    # SLOW_DOWN
    # YIELD_WAIT
    # PASS_INTERSECTION
    # RECOVERY
    # ============================================================
    intersection_supervisor_node = launch_ros.actions.Node(
        package='fishbot_navigation2',
        executable='intersection_supervisor.py',
        name='intersection_supervisor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,

            'map_frame': 'map',
            'robot_frame': 'base_link',
            'dynamic_scan_topic': '/dynamic_scan',
            'world_file_path': world_file_path,
            'moving_model_name': moving_model_name,

            # ===============================
            # 不要太早停车
            # ===============================
            'approach_path_distance': 1.2,
            'near_path_distance': 0.75,

            # 进入动态障碍物运动路径附近后，尽快进入“继续通过”
            'commit_path_half_width': 0.45,
            'exit_path_distance': 1.0,

            # 距离动态障碍物本体的减速 / 停车阈值
            'slow_distance': 0.8,
            'stop_distance': 0.35,

            # 障碍物是否挡住机器人预计穿越点
            'block_path_half_width': 0.35,
            'release_path_distance': 0.65,

            # TTC 判断别太保守
            'ttc_stop_time': 1.8,
            'ttc_slow_time': 3.0,

            # 动态点数量阈值
            'conflict_points_threshold': 3,

            # 速度估计阈值
            'obstacle_velocity_threshold': 0.03,

            # 状态确认时间
            'clear_confirm_time': 0.4,
            'release_confirm_time': 0.5,
            'max_yield_wait_time': 8.0,

            'timer_period': 0.1,
        }]
        
    )

    # ============================================================
    # 4. intersection_speed_gate.py
    #
    # 功能：
    # 订阅 velocity_smoother 输出的 /cmd_vel
    # 订阅 supervisor 输出的 /intersection_decision
    #
    # 根据决策输出：
    # NORMAL_NAVIGATION / PASS_INTERSECTION：正常速度
    # SLOW_DOWN：减速
    # YIELD_WAIT / RECOVERY：停车
    #
    # 输出：
    # /cmd_vel_risk
    # ============================================================
    intersection_speed_gate_node = launch_ros.actions.Node(
        package='fishbot_navigation2',
        executable='intersection_speed_gate.py',
        name='intersection_speed_gate',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,

            # 输入：velocity_smoother 平滑后的速度
            'cmd_vel_in_topic': '/cmd_vel',

            # 输出：经过 TTC / supervisor 门控后的速度
            'cmd_vel_out_topic': '/cmd_vel_risk',

            # 输入：supervisor 决策
            'decision_topic': '/intersection_decision',

            # 减速比例
            'slowdown_ratio': 0.35,

            # 如果你的 intersection_speed_gate.py 支持角速度减速参数，就会生效
            'angular_slowdown_ratio': 0.5,
        }]
    )

    # ============================================================
    # 5. collision_monitor
    #
    # 最后一层安全保护：
    #
    # 输入：/cmd_vel_risk
    # 输出：/cmd_vel_safe
    #
    # nav2_params.yaml 里必须配置：
    #
    # collision_monitor:
    #   ros__parameters:
    #     cmd_vel_in_topic: "cmd_vel_risk"
    #     cmd_vel_out_topic: "cmd_vel_safe"
    # ============================================================
    collision_monitor_node = launch_ros.actions.Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[nav2_param_path],
    )

    # ============================================================
    # 6. collision_monitor lifecycle manager
    #
    # collision_monitor 是 lifecycle node，
    # 需要 lifecycle_manager 将其切换到 active 状态。
    # ============================================================
    collision_monitor_lifecycle_node = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['collision_monitor']
        }]
    )

    # ============================================================
    # 7. RViz
    # ============================================================
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # ============================================================
    # 返回 LaunchDescription
    # ============================================================
    return launch.LaunchDescription([
        # 声明 Launch 参数
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation Gazebo clock if true'
        ),

        launch.actions.DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                fishbot_navigation2_dir,
                'maps',
                'map.yaml'
            ),
            description='Full path to map file to load'
        ),

        launch.actions.DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                fishbot_navigation2_dir,
                'config',
                'nav2_params.yaml'
            ),
            description='Full path to param file to load'
        ),

        launch.actions.DeclareLaunchArgument(
            'world_file_path',
            default_value=os.path.join(
                fishbot_description_dir,
                'world',
                'room.world'
            ),
            description='Full path to Gazebo world file'
        ),

        launch.actions.DeclareLaunchArgument(
            'moving_model_name',
            default_value='',
            description='Moving obstacle model name in world file. Empty means auto select first moving obstacle plugin.'
        ),

        # 启动顺序
        nav2_group,
        dynamic_scan_filter_node,
        intersection_supervisor_node,
        intersection_speed_gate_node,
        collision_monitor_node,
        collision_monitor_lifecycle_node,
        rviz_node,
    ])