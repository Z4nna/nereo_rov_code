from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    control_station_gui = Node(
        package='nereo_pkg',
        executable='camera1_pub',
        parameters=[
            {'fps': 60},
            {'size': (720,480)},
            {'flip_horizontal': True}
        ]
    )
    ld.add_action(control_station_gui)

    ## 
    # need to add the joystick node

    return ld