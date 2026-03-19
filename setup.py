from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wheeltec_semantic_map'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Wheeltec',
    maintainer_email='user@wheeltec.com',
    description='2D Semantic Mapping integrating YOLO and RTAB-Map',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_map_node = wheeltec_semantic_map.semantic_map_node:main',
            'object_navigator_node = wheeltec_semantic_map.object_navigator_node:main',
        ],
    },
)
