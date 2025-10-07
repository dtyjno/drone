from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros_com',
            executable='offboard_control',
            name='offboard_control',
            output='log',
            parameters=[]  # 可添加参数
        )
    ])