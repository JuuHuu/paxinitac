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
