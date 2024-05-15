from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_local_position_publisher=Node(
        package='px4_ros_com',
        executable='vehicle_local_position_publisher',
        output='screen',
        shell=True,
        name='broadcaster1',
        parameters=[
            {'dronename': 'drone1'}
        ],
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        # arguments=["-d", default_rviz_path]
    )
    return LaunchDescription([
        vehicle_local_position_publisher,
        rviz2,
    ])
