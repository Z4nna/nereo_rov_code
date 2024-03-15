from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    camera1 = Node(
        package='nereo_pkg',
        executable='camera1_pub',
        parameters=[
            {'fps': 60},
            {'size': (720,480)},
            {'flip_horizontal': True}
        ]
    )
    ld.add_action(camera1)

    # controller node on the atmega

    # imu node ? 

    # sensors node ? 

    return ld