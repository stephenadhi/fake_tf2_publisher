import os
from glob import glob
from setuptools import setup

package_name = 'fake_tf2_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='master',
    maintainer_email='stephenadhi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_to_odom = fake_tf2_publisher.map_to_odom:main',
            'odom_to_base_footprint = fake_tf2_publisher.odom_to_base_footprint:main'
        ],
    },
)
