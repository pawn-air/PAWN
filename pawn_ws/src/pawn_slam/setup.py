from glob import glob
from setuptools import find_packages, setup

package_name = 'pawn_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='doa',
    maintainer_email='doa@todo.todo',
    description='Camera playback tools and RViz configuration for PAWN SLAM',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'video_camera_publisher = pawn_slam.video_camera_publisher:main',
            'trajectory_publisher = pawn_slam.trajectory_publisher:main',
            'euroc_camera_publisher = pawn_slam.euroc_camera_publisher:main',
        ],
    },
)
