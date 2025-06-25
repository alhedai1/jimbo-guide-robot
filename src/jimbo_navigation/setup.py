from setuptools import find_packages, setup

package_name = 'jimbo_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
        ],
    },
)
