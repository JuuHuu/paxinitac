import os
from setuptools import setup

package_name = 'tactile_raw_point_publisher'
csv_filename = 'PX6AX-GEN3-DP-S2716-Core.csv'
csv_path = os.path.join('..', '..', csv_filename)

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tactile_raw_point_publisher.launch.py']),
        ('share/' + package_name + '/data', [csv_path]),
    ],
    install_requires=['setuptools', 'ament_index_python', 'numpy'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='Publish raw tactile point data from two sensors as PointCloud2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tactile_raw_point_publisher = tactile_raw_point_publisher.node:main',
            'tactile_force_marker_array = tactile_raw_point_publisher.force_marker_node:main',
        ],
    },
)
