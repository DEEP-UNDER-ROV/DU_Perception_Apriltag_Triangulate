import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'jir_image_remapper'
    
    try:
        pkg_share = get_package_share_directory(pkg_name)
        config_dir = os.path.join(pkg_share, 'config')
    except:
        launch_dir = os.path.dirname(os.path.realpath(__file__))
        package_dir = os.path.dirname(launch_dir)
        config_dir = os.path.join(package_dir, 'config')

    # Defaults
    default_left_map = os.path.join(config_dir, 'correctionMap_left.yaml')
    default_right_map = os.path.join(config_dir, 'correctionMap_right.yaml')

    # Arguments
    stereo_arg = DeclareLaunchArgument('stereo', default_value='true')
    
    # Inputs
    left_input_arg = DeclareLaunchArgument(
        'left_input', default_value='/camera/infra1/image_rect_raw'
    )
    right_input_arg = DeclareLaunchArgument(
        'right_input', default_value='/camera/infra2/image_rect_raw'
    )

    # Outputs
    left_output_arg = DeclareLaunchArgument(
        'left_output', default_value='/corrected/left/image_raw'
    )
    right_output_arg = DeclareLaunchArgument(
        'right_output', default_value='/corrected/right/image_raw'
    )

    # Maps
    left_map_arg = DeclareLaunchArgument('left_map', default_value=default_left_map)
    right_map_arg = DeclareLaunchArgument('right_map', default_value=default_right_map)
    
    transport_arg = DeclareLaunchArgument(
        'image_transport', default_value='compressed'
    )

    # Node
    remapper_node = Node(
        package='jir_image_remapper',
        executable='jir_image_remapper',
        name='stereo_remapper_node',
        output='screen',
        parameters=[{
            'stereo': LaunchConfiguration('stereo'),
            'image_transport': LaunchConfiguration('image_transport'),
            'left_map': LaunchConfiguration('left_map'),
            'right_map': LaunchConfiguration('right_map'),
            
            # PASS TOPICS AS PARAMETERS (The Fix)
            'left_input_topic': LaunchConfiguration('left_input'),
            'right_input_topic': LaunchConfiguration('right_input'),
            'left_output_topic': LaunchConfiguration('left_output'),
            'right_output_topic': LaunchConfiguration('right_output'),
        }]
        # Remappings are removed because we are handling it internally now
    )

    return LaunchDescription([
        stereo_arg,
        left_input_arg,
        right_input_arg,
        left_output_arg,
        right_output_arg,
        left_map_arg,
        right_map_arg,
        transport_arg,
        remapper_node,
    ])
