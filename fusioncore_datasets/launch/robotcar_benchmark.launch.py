"""
Oxford Robotcar benchmark launch: plays Robotcar data through FusionCore and
robot_localization EKF simultaneously, then records all outputs to an mcap bag.

Usage:
  # Normal benchmark
  ros2 launch fusioncore_datasets robotcar_benchmark.launch.py \
    data_dir:=/path/to/robotcar/2014-11-18-13-20-12 \
    output_bag:=./benchmarks/robotcar/2014-11-18-13-20-12/bag \
    playback_rate:=5.0 duration_s:=900.0

  # GPS spike test
  ros2 launch fusioncore_datasets robotcar_benchmark.launch.py \
    data_dir:=/path/to/robotcar/2014-11-18-13-20-12 \
    output_bag:=./benchmarks/robotcar/2014-11-18-13-20-12/bag_spike \
    playback_rate:=5.0 duration_s:=300.0 \
    gps_spike_time_s:=120.0 gps_spike_magnitude_m:=500.0

  # GPS outage test
  ros2 launch fusioncore_datasets robotcar_benchmark.launch.py \
    data_dir:=/path/to/robotcar/2014-11-18-13-20-12 \
    output_bag:=./benchmarks/robotcar/2014-11-18-13-20-12/bag_outage \
    playback_rate:=5.0 duration_s:=300.0 \
    gps_outage_start_s:=120.0 gps_outage_duration_s:=45.0

Note: RL-UKF is not included because it diverges to NaN on all tested sequences
(confirmed numerical instability, see NCLT benchmark results).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             TimerAction, LogInfo)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch.actions import RegisterEventHandler, EmitEvent
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    pkg = get_package_share_directory('fusioncore_datasets')
    fc_config  = os.path.join(pkg, 'config', 'robotcar_fusioncore.yaml')
    rl_config  = os.path.join(pkg, 'config', 'rl_ekf.yaml')
    nav_config = os.path.join(pkg, 'config', 'navsat_transform.yaml')

    data_dir     = LaunchConfiguration('data_dir')
    output_bag   = LaunchConfiguration('output_bag')
    rate         = LaunchConfiguration('playback_rate')
    duration     = LaunchConfiguration('duration_s')
    spike_time   = LaunchConfiguration('gps_spike_time_s')
    spike_mag    = LaunchConfiguration('gps_spike_magnitude_m')
    outage_start = LaunchConfiguration('gps_outage_start_s')
    outage_dur   = LaunchConfiguration('gps_outage_duration_s')

    args = [
        DeclareLaunchArgument('data_dir',
                              description='Path to Robotcar sequence directory '
                                          '(containing gps/ins.csv)'),
        DeclareLaunchArgument('output_bag',
                              default_value='./benchmarks/robotcar/bag',
                              description='Output bag path'),
        DeclareLaunchArgument('playback_rate', default_value='3.0',
                              description='Playback speed multiplier'),
        DeclareLaunchArgument('duration_s', default_value='0.0',
                              description='Seconds of data to play (0 = all)'),
        DeclareLaunchArgument('gps_spike_time_s',      default_value='-1.0'),
        DeclareLaunchArgument('gps_spike_magnitude_m', default_value='500.0'),
        DeclareLaunchArgument('gps_outage_start_s',    default_value='-1.0'),
        DeclareLaunchArgument('gps_outage_duration_s', default_value='45.0'),
    ]

    robotcar_player = Node(
        package='fusioncore_datasets',
        executable='robotcar_player.py',
        name='robotcar_player',
        output='screen',
        parameters=[{
            'data_dir':               data_dir,
            'playback_rate':          rate,
            'duration_s':             duration,
            'use_sim_time':           True,
            'gps_spike_time_s':       spike_time,
            'gps_spike_magnitude_m':  spike_mag,
            'gps_outage_start_s':     outage_start,
            'gps_outage_duration_s':  outage_dur,
        }],
    )

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf',
        arguments=['--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        parameters=[{'use_sim_time': True}],
    )

    gps_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='gps_tf',
        arguments=['--z', '1.2',
                   '--frame-id', 'base_link', '--child-frame-id', 'gnss_link'],
        parameters=[{'use_sim_time': True}],
    )

    fusioncore_node = LifecycleNode(
        package='fusioncore_ros',
        executable='fusioncore_node',
        name='fusioncore',
        namespace='',
        output='screen',
        parameters=[fc_config, {'use_sim_time': True}],
    )

    configure_fc = TimerAction(
        period=4.0,
        actions=[
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=lambda a: a == fusioncore_node,
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))
        ],
    )

    activate_fc = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=fusioncore_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=lambda a: a == fusioncore_node,
                    transition_id=Transition.TRANSITION_ACTIVATE,
                ))
            ],
        )
    )

    rl_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='rl_ekf',
        output='screen',
        remappings=[('odometry/filtered', '/rl/odometry')],
        parameters=[rl_config, {'use_sim_time': True}],
    )

    navsat = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        remappings=[
            ('imu/data',          '/imu/data'),
            ('gps/fix',           '/gnss/fix'),
            ('odometry/filtered', '/rl/odometry'),
            ('gps/filtered',      '/rl/gps/filtered'),
            ('odometry/gps',      '/gps/odometry'),
        ],
        parameters=[nav_config, {'use_sim_time': True}],
    )

    recorder = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'record',
                    '-o', output_bag,
                    '/fusion/odom',
                    '/rl/odometry',
                    '/gnss/fix',
                    '/clock',
                ],
                output='screen',
            )
        ],
    )

    return LaunchDescription(args + [
        LogInfo(msg='Starting Oxford Robotcar benchmark...'),
        robotcar_player,
        imu_tf,
        gps_tf,
        fusioncore_node,
        configure_fc,
        activate_fc,
        rl_ekf,
        navsat,
        recorder,
    ])
