import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
            name='seekthermal',
            namespace='libseek',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='libseek-thermal',
                    plugin='SeekThermalRos',
                    name='seekthermal',
                    parameters=[{'rotate90': 0, 'device_index': 0, 'is_seek_pro': True}])
            ],
            output='screen'            
    )

    return launch.LaunchDescription([container])