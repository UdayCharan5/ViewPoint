from setuptools import setup
import os
from glob import glob

package_name = 'oculus_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # These two lines fix the colcon warnings
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch and config files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'sonar_node = oculus_driver.sonar_ros_node:main',
            'tf_static  = oculus_driver.tf_broadcaster:main',
        ],
    },
)
