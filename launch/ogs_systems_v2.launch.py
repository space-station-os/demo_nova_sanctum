from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # params_file = os.path.join(
    #     get_package_share_directory('demo_nova_sanctum'),
    #     'config',
    #     'ars_sys.yaml'
    # )

    return LaunchDescription([
        Node(
            package='demo_nova_sanctum',
            executable='water_pub',
            name='potable_water_pub',
            output='screen',
            # parameters=[params_file],
            emulate_tty=True
        ),

        Node(
            package='demo_nova_sanctum',
            executable='deionization_bed',
            name='contaminant_removal',
            output='screen',
            emulate_tty=True
        ),
        
        Node(
            package='demo_nova_sanctum',
            executable='electrolysis',
            name='electrolysis_h2_o2',
            output='screen',
            emulate_tty=True
        ),
        
        # Node(
        #     package='demo_nova_sanctum',
        #     executable='sabatier',
        #     name='sabatier_reaction',
        #     output='screen',
        #     emulate_tty=True
        # ),
        
    
    ])
