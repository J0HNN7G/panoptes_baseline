from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import csv
import os

def generate_transform_nodes_from_csv(transforms_file):
    transform_nodes = []
    # Open and parse the CSV file
    with open(transforms_file, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            # Get the values from the CSV row
            tag_family = row['tag_family']
            tag_id = int(row['tag_id'])
            trans_x = row['trans_x']
            trans_y = row['trans_y']
            trans_z = row['trans_z']
            rot_x = row['rot_x']
            rot_y = row['rot_y']
            rot_z = row['rot_z']
            rot_w = row['rot_w']
            
            # Format the child-frame-id with tag family and id
            child_frame_id = f"{tag_family}:{tag_id}"
            
            # Create a dynamic node name using tag_family and tag_id
            node_name = f"static_transform_publisher_{tag_family}_{tag_id}"

            # Create a Node for static transform publisher
            # decide on whether each marker gets a seperate world frame or not
            node = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=[
                    trans_x, trans_y, trans_z, rot_x, rot_y, rot_z, rot_w,
                    'map', child_frame_id
                ],
                name=node_name
            )
            
            # Append the node to the list of transform nodes
            transform_nodes.append(node)
    
    return transform_nodes

def create_nodes(context, *args, **kwargs):
    # Resolve the file path from the launch configuration
    transforms_file = LaunchConfiguration('transforms_file').perform(context)
    
    if os.path.exists(transforms_file):
        return generate_transform_nodes_from_csv(transforms_file)
    else:
        raise FileNotFoundError(f"CSV file '{transforms_file}' not found!")

def generate_launch_description():
    # Declare the launch argument for the transforms file
    transforms_file_arg = DeclareLaunchArgument(
        'transforms_file',
        default_value='/path/to/transforms.csv',
        description='Path to the CSV file containing transform data'
    )

    # Use OpaqueFunction to call create_nodes dynamically
    dynamic_node_generation = OpaqueFunction(function=create_nodes)

    return LaunchDescription([
        transforms_file_arg,
        dynamic_node_generation
    ])
