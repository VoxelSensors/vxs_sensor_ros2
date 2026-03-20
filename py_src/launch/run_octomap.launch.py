from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': 0.05,              # Matches your Voxel Decimator perfectly!
            'frame_id': 'odom',              # The fixed global frame
            'base_frame_id': 'sensor',    # Your sensor frame
            'sensor_model.max_range': 2.0,   # Ignore noise past 5 meters
            'filter_ground': True           # Don't delete your wire thinking it's the floor
        }],
        remappings=[
            ('cloud_in', '/pcloud/voxelized') # Listen to your green spheres
        ]
    )

    return LaunchDescription([octomap_node])