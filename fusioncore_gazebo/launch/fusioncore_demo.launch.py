import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction,
    RegisterEventHandler, EmitEvent,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    gz_pkg  = get_package_share_directory("fusioncore_gazebo")
    fc_pkg  = get_package_share_directory("fusioncore_ros")
    rl_yaml = os.path.join(gz_pkg, "config", "rl_ekf_gazebo.yaml")
    fc_yaml = os.path.join(gz_pkg, "config", "fusioncore_gazebo.yaml")
    rviz_cfg = os.path.join(gz_pkg, "config", "demo.rviz")
    world   = os.path.join(gz_pkg, "worlds", "fusioncore_outdoor.sdf")
    models  = os.path.join(gz_pkg, "models")

    # LifecycleNode needs a local reference for the event handlers
    fusioncore_node = LifecycleNode(
        package="fusioncore_ros",
        executable="fusioncore_node",
        name="fusioncore",
        namespace="",
        output="screen",
        parameters=[fc_yaml],
    )

    configure_cmd = TimerAction(period=15.0, actions=[
        EmitEvent(event=ChangeState(
            lifecycle_node_matcher=lambda action: action == fusioncore_node,
            transition_id=Transition.TRANSITION_CONFIGURE,
        )),
    ])

    activate_cmd = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=fusioncore_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=lambda action: action == fusioncore_node,
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    return LaunchDescription([
        # ── Args ──────────────────────────────────────────────────────
        DeclareLaunchArgument("spike_at_s",       default_value="30.0",
            description="Seconds after GPS node start to inject spike"),
        DeclareLaunchArgument("spike_duration_s",  default_value="6.0",
            description="How long the spike lasts (s)"),
        DeclareLaunchArgument("spike_dx_m",        default_value="50.0",
            description="Spike offset east (m): big enough to trigger chi2 rejection"),
        DeclareLaunchArgument("spike_dy_m",        default_value="0.0"),
        DeclareLaunchArgument("rviz",              default_value="true",
            description="Launch RViz"),

        # ── Gazebo sim ────────────────────────────────────────────────
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world],
            additional_env={"GZ_SIM_RESOURCE_PATH": models},
            output="screen",
        ),

        # ── ROS-Gazebo bridge ─────────────────────────────────────────
        # override_timestamps_with_wall_time: avoids sim-time vs wall-time
        # mismatch that would prevent FusionCore from fusing anything.
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_bridge",
            output="screen",
            parameters=[{
                "override_timestamps_with_wall_time": True,
                "expand_gz_topic_names": True,
            }],
            remappings=[
                ("/fusioncore_robot/imu_link/imu_sensor", "/imu/data"),
            ],
            arguments=[
                "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
                "/world/fusioncore_outdoor/pose/info"
                    "@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
                "/odom/wheels@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            ],
        ),

        # ── Static TFs ────────────────────────────────────────────────
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_tf",
            arguments=["--x", "0", "--y", "0", "--z", "0.1",
                       "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", "base_link", "--child-frame-id", "imu_link"],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_tf_gz",
            arguments=["--x", "0", "--y", "0", "--z", "0.1",
                       "--roll", "0", "--pitch", "0", "--yaw", "0",
                       "--frame-id", "base_link",
                       "--child-frame-id", "fusioncore_robot/imu_link/imu_sensor"],
        ),

        # ── GPS publisher with spike injection ────────────────────────
        Node(
            package="fusioncore_gazebo",
            executable="gz_pose_to_gps",
            name="gz_pose_to_gps",
            output="screen",
            parameters=[{
                "world_name":       "fusioncore_outdoor",
                "spike_at_s":       LaunchConfiguration("spike_at_s"),
                "spike_duration_s": LaunchConfiguration("spike_duration_s"),
                "spike_dx_m":       LaunchConfiguration("spike_dx_m"),
                "spike_dy_m":       LaunchConfiguration("spike_dy_m"),
            }],
        ),

        # ── robot_localization EKF (no rejection threshold: diverges on spike) ─
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="rl_ekf",
            output="screen",
            parameters=[rl_yaml],
            remappings=[("odometry/filtered", "/odometry/filtered")],
        ),

        fusioncore_node,
        configure_cmd,
        activate_cmd,

        # ── Circle driver: starts moving at t=18s via internal timer ──
        Node(
            package="fusioncore_gazebo",
            executable="circle_driver",
            name="circle_driver",
            output="screen",
            parameters=[{
                "linear_speed":  0.8,
                "radius":        12.0,
                "start_delay_s": 18.0,
            }],
        ),

        # ── Path publisher: trajectory lines for RViz ─────────────────
        Node(
            package="fusioncore_gazebo",
            executable="path_publisher",
            name="path_publisher",
            output="screen",
        ),

        # ── RViz ──────────────────────────────────────────────────────
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_cfg],
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
