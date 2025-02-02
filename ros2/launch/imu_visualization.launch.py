from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    package_name = 'lsm6dso32_driver'
    
    # Declare launch arguments
    device_path_arg = DeclareLaunchArgument(
        'device_path',
        default_value='/dev/i2c-1',
        description='Path to I2C device'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU messages'
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare(package_name),
        'config',
        'imu_visualization.rviz'
    ])

    return LaunchDescription([
        device_path_arg,
        frame_id_arg,
        
        Node(
            package=package_name,
            executable='imu_node_exec',
            name='imu_node',
            parameters=[{
                'device_path': LaunchConfiguration('device_path'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rate': 200.0,
                'publish_tf': True
            }],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
