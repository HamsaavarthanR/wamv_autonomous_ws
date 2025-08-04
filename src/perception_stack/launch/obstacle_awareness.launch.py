# Import libraries
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()
    # Obstacle Detection Node
    obstacle_detection_node = Node(
        package='perception_stack',
        executable='obstacle_detector',
        parameters=[{'MODEL_PATH': PathJoinSubstitution([
                    FindPackageShare('perception_stack'),
                    'my_model',
                    'yolo_modd.pt'
                ]),
            'use_sim_time': True}],
    )
        
    # Sensor Fusion Node
    sensor_fusion_node = Node(
        package='perception_stack',
        executable='sensor_fusion',
        parameters=[{'config_path': PathJoinSubstitution([
                    FindPackageShare('perception_stack'),
                    'config',
                    'sensor_params.yaml'
                ]),
                     'use_sim_time': True}],
    )

    ld.add_action(obstacle_detection_node)
    ld.add_action(sensor_fusion_node) 
    return ld

