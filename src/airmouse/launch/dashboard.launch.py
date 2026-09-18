import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rosbridge_pkg = get_package_share_directory('rosbridge_server')
    rosbridge_launch = os.path.join(rosbridge_pkg, 'launch', 'rosbridge_websocket_launch.xml')

    http_port_arg = DeclareLaunchArgument(
        'http_port',
        default_value='8080',
        description='HTTP server port for web GCS dashboard'
    )
    ws_port_arg = DeclareLaunchArgument(
        'ws_port',
        default_value='9090',
        description='WebSocket bridge port for rosbridge'
    )

    return LaunchDescription([
        http_port_arg,
        ws_port_arg,
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch),
            launch_arguments={'port': LaunchConfiguration('ws_port')}.items()
        ),
        Node(
            package='airmouse',
            executable='dashboard_server',
            name='dashboard_server',
            output='screen',
            parameters=[{'port': LaunchConfiguration('http_port')}]
        )
    ])
