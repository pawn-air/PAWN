import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    get = lambda n: LaunchConfiguration(n).perform(context)
    drone = int(get('drone_id'))
    source = get('camera_source')
    if drone < 1 or source not in ('euroc', 'video'):
        raise ValueError('Requires drone_id >= 1 and camera_source=euroc|video')
    root = os.path.expanduser(get('orb_slam3_dir'))
    dataset = os.path.expanduser(get('dataset_path'))
    video = os.path.expanduser(get('video_path'))
    cfg = os.path.expanduser(get('settings_path'))
    if not cfg:
        cfg = (os.path.join(root, 'Examples/Monocular/EuRoC.yaml') if source == 'euroc'
               else os.path.join(get_package_share_directory('pawn_orbslam3'), 'config/s24_test.yaml'))
    vocab = os.path.join(root, 'Vocabulary/ORBvoc.txt')
    for label, item in [('ORB vocabulary', vocab), ('camera settings', cfg)]:
        if not os.path.isfile(item):
            raise FileNotFoundError(f'{label}: {item}')
    if source == 'euroc' and not os.path.isfile(os.path.join(dataset,'mav0/cam0/data.csv')):
        raise FileNotFoundError('EuRoC cam0 data.csv missing under '+dataset)
    if source == 'video' and not os.path.isfile(video):
        raise FileNotFoundError('Video not found: '+video)
    topic = f'/drone_{drone}/camera/image_raw'
    slam = Node(package='pawn_orbslam3', executable='orb_slam_node', name='vi_slam',
                namespace=f'drone_{drone}', output='screen', parameters=[{
                    'drone_id': drone, 'image_topic': topic,
                    'publish_path': ParameterValue(LaunchConfiguration('publish_path'), value_type=bool),
                    'vocabulary_path': vocab, 'settings_path': cfg}])
    publisher = Node(package='pawn_slam',
                     executable='euroc_camera_publisher' if source == 'euroc' else 'video_camera_publisher',
                     namespace=f'drone_{drone}', output='screen',
                     parameters=[{'drone_id': drone, 'image_topic': topic,
                                  **({'dataset_path': dataset} if source == 'euroc'
                                     else {'video_path': video})}])
    actions = [slam, TimerAction(period=15.0, actions=[publisher])]
    if get('enable_mapping').lower() in ('true', '1', 'yes'):
        mapper = Node(
            package='pawn_orbslam3',
            executable='octomap_mapper_node',
            name='octomap_mapper',
            namespace=f'drone_{drone}',
            output='screen',
            parameters=[{
                'drone_id': drone,
                'resolution': float(get('octomap_resolution')),
            }]
        )
        actions.insert(1, mapper)
    if get('rviz').lower() in ('true','1','yes'):
        rviz_cfg = os.path.join(get_package_share_directory('pawn_slam'), 'rviz/pawn_slam.rviz')
        actions.insert(1, Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg, '-f', f'drone_{drone}/slam_map'],
                               remappings=[(f'/drone_1/{suffix}', f'/drone_{drone}/{suffix}')
                                           for suffix in ('camera/image_raw', 'slam/pose', 'path', 'slam/map_points',
                                                          'slam_points_filtered', 'occupied_cells_vis_array',
                                                          'map/frontier_markers', 'octomap_binary')], output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('publish_path', default_value='true'),
        DeclareLaunchArgument('enable_mapping', default_value='true', description='Start Kisi B OctoMap 3D mapper node'),
        DeclareLaunchArgument('octomap_resolution', default_value='0.08', description='Octree voxel size in meters'),
        DeclareLaunchArgument('camera_source', default_value='euroc'),
        DeclareLaunchArgument('drone_id', default_value='1'),
        DeclareLaunchArgument('dataset_path', default_value='', description='Required for EuRoC: absolute MH01 directory'),
        DeclareLaunchArgument('video_path', default_value='', description='Required for video: mp4 path'),
        DeclareLaunchArgument('settings_path', default_value='', description='Override source-specific camera YAML'),
        DeclareLaunchArgument('orb_slam3_dir', default_value=os.environ.get('ORB_SLAM3_DIR',os.path.expanduser('~/PAWN/ORB_SLAM3'))),
        DeclareLaunchArgument('rviz', default_value='true', description='Show selected drone local SLAM map'),
        OpaqueFunction(function=start),
    ])
