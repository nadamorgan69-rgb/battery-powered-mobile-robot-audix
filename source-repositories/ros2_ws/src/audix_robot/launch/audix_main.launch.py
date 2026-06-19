from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    mock_ir = LaunchConfiguration("mock_ir")
    mock_gpio = LaunchConfiguration("mock_gpio")
    ir_enabled = LaunchConfiguration("ir_enabled")
    ir_logic = LaunchConfiguration("ir_logic")
    init_imu_on_start = LaunchConfiguration("init_imu_on_start")
    reset_odom_on_start = LaunchConfiguration("reset_odom_on_start")
    dashboard_port = LaunchConfiguration("dashboard_port")
    allow_placeholder_audit = LaunchConfiguration("allow_placeholder_audit")
    camera_enabled = LaunchConfiguration("camera_enabled")
    camera_index = LaunchConfiguration("camera_index")
    vision_enabled = LaunchConfiguration("vision_enabled")
    vision_confidence = LaunchConfiguration("vision_confidence")
    vision_target_count = LaunchConfiguration("vision_target_count")
    vision_scan_settle = LaunchConfiguration("vision_scan_settle")
    audit_side_1_shelf_id = LaunchConfiguration("audit_side_1_shelf_id")
    audit_side_2_shelf_id = LaunchConfiguration("audit_side_2_shelf_id")
    audit_default_expected_count = LaunchConfiguration("audit_default_expected_count")
    traffic_green_pin = LaunchConfiguration("traffic_green_pin")
    traffic_yellow_pin = LaunchConfiguration("traffic_yellow_pin")
    traffic_red_pin = LaunchConfiguration("traffic_red_pin")
    traffic_active_high = LaunchConfiguration("traffic_active_high")
    map_width = LaunchConfiguration("map_width_cm")
    map_height = LaunchConfiguration("map_height_cm")
    map_spawn_x = LaunchConfiguration("map_spawn_x_cm")
    map_spawn_y = LaunchConfiguration("map_spawn_y_cm")
    map_top_travel_y = LaunchConfiguration("map_top_travel_y_cm")
    map_audit_y = LaunchConfiguration("map_audit_y_cm")
    map_lane_1_x = LaunchConfiguration("map_lane_1_x_cm")
    map_lane_2_x = LaunchConfiguration("map_lane_2_x_cm")
    map_scan_heading_1 = LaunchConfiguration("map_scan_heading_1_deg")
    map_scan_heading_2 = LaunchConfiguration("map_scan_heading_2_deg")
    map_front_avoidance_bias_1 = LaunchConfiguration("map_front_avoidance_bias_1")
    map_front_avoidance_bias_2 = LaunchConfiguration("map_front_avoidance_bias_2")
    map_shelf_x_min = LaunchConfiguration("map_shelf_x_min_cm")
    map_shelf_x_max = LaunchConfiguration("map_shelf_x_max_cm")
    map_shelf_y_min = LaunchConfiguration("map_shelf_y_min_cm")
    map_shelf_y_max = LaunchConfiguration("map_shelf_y_max_cm")
    front_corner_blind_pass_distance = LaunchConfiguration("front_corner_blind_pass_distance_m")
    front_corner_blind_pass_timeout = LaunchConfiguration("front_corner_blind_pass_timeout_s")
    front_side_blind_pass_distance = LaunchConfiguration("front_side_blind_pass_distance_m")
    front_side_blind_pass_timeout = LaunchConfiguration("front_side_blind_pass_timeout_s")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="audix"),
            DeclareLaunchArgument("port", default_value="/dev/ttyAMA0"),
            DeclareLaunchArgument("baud", default_value="115200"),
            DeclareLaunchArgument("mock_ir", default_value="false"),
            DeclareLaunchArgument("mock_gpio", default_value="false"),
            DeclareLaunchArgument("ir_enabled", default_value="true"),
            DeclareLaunchArgument("ir_logic", default_value="baseline"),
            DeclareLaunchArgument("init_imu_on_start", default_value="true"),
            DeclareLaunchArgument("reset_odom_on_start", default_value="true"),
            DeclareLaunchArgument("dashboard_port", default_value="8080"),
            DeclareLaunchArgument("allow_placeholder_audit", default_value="false"),
            DeclareLaunchArgument("camera_enabled", default_value="true"),
            DeclareLaunchArgument("camera_index", default_value="0"),
            DeclareLaunchArgument("vision_enabled", default_value="true"),
            DeclareLaunchArgument("vision_confidence", default_value="0.5"),
            DeclareLaunchArgument("vision_target_count", default_value="2"),
            DeclareLaunchArgument("vision_scan_settle", default_value="0.5"),
            DeclareLaunchArgument("audit_side_1_shelf_id", default_value="indomie"),
            DeclareLaunchArgument("audit_side_2_shelf_id", default_value="fruit_rings_cereal"),
            DeclareLaunchArgument("audit_default_expected_count", default_value="2"),
            DeclareLaunchArgument("traffic_green_pin", default_value="16"),
            DeclareLaunchArgument("traffic_yellow_pin", default_value="20"),
            DeclareLaunchArgument("traffic_red_pin", default_value="21"),
            DeclareLaunchArgument("traffic_active_high", default_value="true"),
            DeclareLaunchArgument("map_width_cm", default_value="300.0"),
            DeclareLaunchArgument("map_height_cm", default_value="210.0"),
            DeclareLaunchArgument("map_spawn_x_cm", default_value="25.0"),
            DeclareLaunchArgument("map_spawn_y_cm", default_value="-29.0"),
            DeclareLaunchArgument("map_top_travel_y_cm", default_value="-29.0"),
            DeclareLaunchArgument("map_audit_y_cm", default_value="-105.0"),
            DeclareLaunchArgument("map_lane_1_x_cm", default_value="60.0"),
            DeclareLaunchArgument("map_lane_2_x_cm", default_value="240.0"),
            DeclareLaunchArgument("map_scan_heading_1_deg", default_value="-90.0"),
            DeclareLaunchArgument("map_scan_heading_2_deg", default_value="90.0"),
            DeclareLaunchArgument("map_front_avoidance_bias_1", default_value="-1"),
            DeclareLaunchArgument("map_front_avoidance_bias_2", default_value="1"),
            DeclareLaunchArgument("map_shelf_x_min_cm", default_value="130.0"),
            DeclareLaunchArgument("map_shelf_x_max_cm", default_value="170.0"),
            DeclareLaunchArgument("map_shelf_y_min_cm", default_value="-130.0"),
            DeclareLaunchArgument("map_shelf_y_max_cm", default_value="-80.0"),
            DeclareLaunchArgument("front_corner_blind_pass_distance_m", default_value="0.25"),
            DeclareLaunchArgument("front_corner_blind_pass_timeout_s", default_value="4.0"),
            DeclareLaunchArgument("front_side_blind_pass_distance_m", default_value="0.35"),
            DeclareLaunchArgument("front_side_blind_pass_timeout_s", default_value="4.0"),
            Node(
                package="micro_ros_agent",
                executable="micro_ros_agent",
                name="micro_ros_agent",
                output="screen",
                arguments=["serial", "--dev", port, "--baudrate", baud],
            ),
            Node(
                package="audix_robot",
                executable="micro_ros_base_node",
                name="micro_ros_base",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "frame_id": "odom",
                        "base_frame_id": "base_footprint",
                        "publish_odom": True,
                        "publish_tf": True,
                        "init_imu_on_start": ParameterValue(init_imu_on_start, value_type=bool),
                        "reset_odom_on_start": ParameterValue(reset_odom_on_start, value_type=bool),
                    }
                ],
            ),
            Node(
                package="audix_robot",
                executable="gpio_hardware_node",
                name="gpio_hardware",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "mock_gpio": ParameterValue(mock_gpio, value_type=bool),
                        "mock_ir": ParameterValue(mock_ir, value_type=bool),
                        "ir_enabled": ParameterValue(ir_enabled, value_type=bool),
                        "ir_logic": ir_logic,
                        "traffic_green_pin": ParameterValue(traffic_green_pin, value_type=int),
                        "traffic_yellow_pin": ParameterValue(traffic_yellow_pin, value_type=int),
                        "traffic_red_pin": ParameterValue(traffic_red_pin, value_type=int),
                        "traffic_active_high": ParameterValue(traffic_active_high, value_type=bool),
                    }
                ],
            ),
            Node(
                package="audix_robot",
                executable="robot_manager_node",
                name="robot_manager",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "allow_placeholder_audit": ParameterValue(allow_placeholder_audit, value_type=bool),
                        "vision_scan_settle_s": ParameterValue(vision_scan_settle, value_type=float),
                        "audit_side_1_shelf_id": audit_side_1_shelf_id,
                        "audit_side_2_shelf_id": audit_side_2_shelf_id,
                        "audit_default_expected_count": ParameterValue(
                            audit_default_expected_count,
                            value_type=int,
                        ),
                        "map_width_cm": ParameterValue(map_width, value_type=float),
                        "map_height_cm": ParameterValue(map_height, value_type=float),
                        "map_spawn_x_cm": ParameterValue(map_spawn_x, value_type=float),
                        "map_spawn_y_cm": ParameterValue(map_spawn_y, value_type=float),
                        "map_top_travel_y_cm": ParameterValue(map_top_travel_y, value_type=float),
                        "map_audit_y_cm": ParameterValue(map_audit_y, value_type=float),
                        "map_lane_1_x_cm": ParameterValue(map_lane_1_x, value_type=float),
                        "map_lane_2_x_cm": ParameterValue(map_lane_2_x, value_type=float),
                        "map_scan_heading_1_deg": ParameterValue(map_scan_heading_1, value_type=float),
                        "map_scan_heading_2_deg": ParameterValue(map_scan_heading_2, value_type=float),
                        "map_front_avoidance_bias_1": ParameterValue(map_front_avoidance_bias_1, value_type=int),
                        "map_front_avoidance_bias_2": ParameterValue(map_front_avoidance_bias_2, value_type=int),
                        "map_shelf_x_min_cm": ParameterValue(map_shelf_x_min, value_type=float),
                        "map_shelf_x_max_cm": ParameterValue(map_shelf_x_max, value_type=float),
                        "map_shelf_y_min_cm": ParameterValue(map_shelf_y_min, value_type=float),
                        "map_shelf_y_max_cm": ParameterValue(map_shelf_y_max, value_type=float),
                        "front_corner_blind_pass_distance_m": ParameterValue(
                            front_corner_blind_pass_distance, value_type=float
                        ),
                        "front_corner_blind_pass_timeout_s": ParameterValue(
                            front_corner_blind_pass_timeout, value_type=float
                        ),
                        "front_side_blind_pass_distance_m": ParameterValue(
                            front_side_blind_pass_distance, value_type=float
                        ),
                        "front_side_blind_pass_timeout_s": ParameterValue(
                            front_side_blind_pass_timeout, value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="warehouse_vision",
                executable="webcam_node",
                name="webcam",
                namespace=namespace,
                output="screen",
                condition=IfCondition(camera_enabled),
                parameters=[
                    {
                        "camera_index": ParameterValue(camera_index, value_type=int),
                        "image_topic": "image_raw",
                    }
                ],
            ),
            Node(
                package="warehouse_vision",
                executable="vision_audit_node",
                name="vision_audit",
                namespace=namespace,
                output="screen",
                condition=IfCondition(vision_enabled),
                parameters=[
                    {
                        "image_topic": "image_raw",
                        "scan_service": "scan_shelf",
                        "confidence_threshold": ParameterValue(vision_confidence, value_type=float),
                        "target_count": ParameterValue(vision_target_count, value_type=int),
                    }
                ],
            ),
            Node(
                package="audix_robot",
                executable="web_dashboard_node",
                name="web_dashboard",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "host": "0.0.0.0",
                        "port": ParameterValue(dashboard_port, value_type=int),
                    }
                ],
            ),
        ]
    )
