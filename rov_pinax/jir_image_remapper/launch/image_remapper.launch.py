from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    left_map_arg = DeclareLaunchArgument(
        'left_map',
        default_value='/path/to/correctionMap.yaml',
        description='Path to the left camera correction map'
    )

    remapper_node = Node(
        package='jir_image_remapper',
        executable='jir_image_remapper',
        name='jir_image_remapper',
        output='screen',
        parameters=[{
            'left_map': LaunchConfiguration('left_map')
        }],
        remappings=[
            ('in_left', 'camera/left/image_raw'),
            ('out_left_rect/image_raw', 'rectified/left/image_raw'),
            ('out_left_rect/camera_info', 'rectified/left/camera_info'),
        ]
    )

    return LaunchDescription([
        left_map_arg,
        remapper_node,
    ])
