import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    drone = int(get('drone_id'))
    if drone < 1:
        raise ValueError('drone_id must be >= 1')
    root = os.path.expanduser(get('orb_slam3_dir'))
    vocabulary = os.path.expanduser(get('vocabulary_path')) or os.path.join(root, 'Vocabulary/ORBvoc.txt')
    settings = os.path.expanduser(get('settings_path'))
    for label, filename in [('vocabulary_path', vocabulary), ('settings_path', settings)]:
        if not os.path.isfile(filename):
            raise FileNotFoundError(f'{label}: {filename}. Specify your camera calibration.')
    nodes = [Node(package='pawn_orbslam3', executable='orb_slam_node',
                  name='vi_slam', namespace=f'drone_{drone}', output='screen',
                  parameters=[{'drone_id': drone, 'image_topic': get('image_topic'),
                               'publish_path': ParameterValue(LaunchConfiguration('publish_path'), value_type=bool),
                               'vocabulary_path': vocabulary, 'settings_path': settings}])]
    if get('enable_mapping').lower() in ('true', '1', 'yes'):
        nodes.append(Node(
            package='pawn_orbslam3',
            executable='octomap_mapper_node',
            name='octomap_mapper',
            namespace=f'drone_{drone}',
            output='screen',
            parameters=[{
                'drone_id': drone,
                'resolution': float(get('octomap_resolution')),
            }]
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('publish_path', default_value='true'),
        DeclareLaunchArgument('enable_mapping', default_value='true', description='Start Kisi B OctoMap 3D mapper node'),
        DeclareLaunchArgument('octomap_resolution', default_value='0.08', description='Octree voxel size in meters'),
        DeclareLaunchArgument('drone_id', default_value='1'),
        DeclareLaunchArgument('image_topic', default_value='', description='Empty = /drone_<id>/camera/image_raw'),
        DeclareLaunchArgument('orb_slam3_dir', default_value=os.environ.get('ORB_SLAM3_DIR', os.path.expanduser('~/PAWN/ORB_SLAM3'))),
        DeclareLaunchArgument('vocabulary_path', default_value='', description='Empty = ORB_SLAM3_DIR/Vocabulary/ORBvoc.txt'),
        DeclareLaunchArgument('settings_path', default_value='', description='Required camera calibration YAML'),
        OpaqueFunction(function=start),
    ])
