from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    robomaster_name = LaunchConfiguration('robomaster_name').perform(context)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                 PathJoinSubstitution([
                    FindPackageShare('depthai_examples'), 
                    'launch',
                    'panoptes_baseline.launch.py'
                ])
            ]),
            launch_arguments={'namespace': robomaster_name}.items()
        ),

        Node(
            package='panoptes_baseline',
            executable='detection_tracker_filter',
            name='detection_tracker_filter',
            namespace=robomaster_name,
            parameters=[{
                'input_topic': 'oak/nn/yolov4_Spatial_tracklets',
                'namespace': robomaster_name,
                'max_humans': 5
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                  PathJoinSubstitution([
                    FindPackageShare('apriltag_ros'),
                    'launch',
                    'panoptes_baseline.launch.py'
                ])
            ]),
            launch_arguments={
                'image_rect': f'/{robomaster_name}/oak/rgb/image_rect',
                'camera_info': f'/{robomaster_name}/oak/rgb/camera_info',
                'namespace': robomaster_name,
                'params_file': PathJoinSubstitution([
                    FindPackageShare('apriltag_ros'),
                    'cfg',
                    'panoptes_baseline.yaml'
                ])
            }.items()
        ),
    ]

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'robomaster_name',
            default_value='',
            description='Namespace for the robot'
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])