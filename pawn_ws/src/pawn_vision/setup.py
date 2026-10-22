from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pawn_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'weights'), ['weights/download_weights.sh']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nijat Mukhtarov',
    maintainer_email='mukhog97@icloud.com',
    description='PAWN Multi-Drone Search and Rescue (SAR) Perception, Detection and Risk Assessment Engine',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = pawn_vision.vision_node:main',
            'video_stream = pawn_vision.sar_video_pipeline:main',
            'image_analyzer = pawn_vision.sar_master_pipeline:main',
        ],
    },
)
