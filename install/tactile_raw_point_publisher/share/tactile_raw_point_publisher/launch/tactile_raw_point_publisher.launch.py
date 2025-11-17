from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Load the extra URDF as text ---
    sensor_urdf_path = os.path.join(
        get_package_share_directory('tactile_raw_point_publisher'),
        'urdf',
        'Sensor_holer_URDF.urdf',
    )
    with open(sensor_urdf_path, 'r') as f:
        sensor_urdf_xml = f.read()
        
    # parent frame: robot flange / tool frame (e.g. "tool0" or "flange")
    parent_frame = 'tool0'
    # child frame: root of your sensor holder URDF
    child_frame = 'sensor_base'

    # ⚙️ Static TF from flange -> sensor_base
    static_tf_flange_to_sensor_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='flange_to_sensor_base_tf',
        # arguments: x y z roll pitch yaw parent_frame child_frame
        arguments=[
            '0.0', '0.0', '0.0',      # xyz in meters
            '0.0', '0.0', '0.0',      # rpy in radians
            parent_frame,
            child_frame,
        ],
        output='screen',
    )
        
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='tactile_sensor_state_publisher',
            output='screen',
            parameters=[{'robot_description': sensor_urdf_xml}],
        ),
        
        Node(
            package='tactile_raw_point_publisher',
            executable='tactile_raw_point_publisher',
            name='tactile_raw_point_publisher',
            output='screen',
            parameters=[
                {'publish_rate_hz': 100.0},
                {'sensor_1_id': 1},
                {'sensor_3_id': 3},
                {'sensor_1_frame': 'sensor_1_link'},
                {'sensor_3_frame': 'sensor_3_link'},
            ],
        ),
        Node(
            package='tactile_raw_point_publisher',
            executable='tactile_force_marker_array',
            name='tactile_force_marker_array_sensor1',
            output='screen',
            parameters=[
                {'pointcloud_topic': '/tactile/sensor_1/raw_point_data'},
                {'marker_topic': '/tactile/sensor_1/force_markers'},
                {'default_frame': 'sensor_1_link'},
            ],
        ),
        Node(
            package='tactile_raw_point_publisher',
            executable='tactile_force_marker_array',
            name='tactile_force_marker_array_sensor3',
            output='screen',
            parameters=[
                {'pointcloud_topic': '/tactile/sensor_3/raw_point_data'},
                {'marker_topic': '/tactile/sensor_3/force_markers'},
                {'default_frame': 'sensor_3_link'},
            ],
        ),
    ])
