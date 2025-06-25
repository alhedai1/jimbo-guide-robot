from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'jimbo_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        ('share/ament_index/resource_index/packages',
            ['resource/jimbo_navigation']),
        ('share/jimbo_navigation/launch', [
            'launch/guide_robot.launch.py',
            'launch/robot_description.launch.py',
            'launch/test_transforms.launch.py',
            'launch/slam.launch.py',
            'launch/navigation.launch.py'
        ]),
        # Include maps directory
        (os.path.join('share', package_name, 'maps'), []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jimbo',
    maintainer_email='jimbo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_monitor = jimbo_navigation.safety_monitor:main',
            'user_follower = jimbo_navigation.user_follower:main',
            'tf_test = jimbo_navigation.tf_test:main',
        ],
    },
)
