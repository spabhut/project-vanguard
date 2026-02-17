from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'six_wheel_rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include URDF files
        (os.path.join('share', package_name, 'model'), glob('model/*.xacro')),
        # Include Config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),        
        #Include World files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sparsh',
    maintainer_email='spabhut286@gmail.com',
    description='6-wheel rover simulation package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = six_wheel_rover.controller:main',
            'teleop = six_wheel_rover.teleop:main',
        ],
    },
)
