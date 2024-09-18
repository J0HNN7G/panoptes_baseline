import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'panoptes_baseline'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'cfg'), glob(os.path.join('cfg', '*.*'))),
    ],
    install_requires=['setuptools', 'rclpy', 'vision_msgs', 'geometry_msgs', 'tf2_ros_py'],
    zip_safe=True,
    maintainer='jgf',
    maintainer_email='jonathan.frennert@gmail.com',
    description='Nodes and launch files for panoptes baseline',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'detection_filter = panoptes_baseline.detection_filter:main',
                'detection_tracker_filter = panoptes_baseline.detection_tracker_filter:main',
        ],
    },
)
