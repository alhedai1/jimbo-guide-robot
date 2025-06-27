from setuptools import setup, find_packages

package_name = 'motor_interface_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # 이게 핵심!
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/motor_serial.launch.py']),
        ('share/' + package_name, ['resource/' + package_name]),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'jimbo_msgs',
    ],
    zip_safe=True,
    maintainer='jimbo',
    maintainer_email='jimbo@todo.todo',
    description='Motor interface Python package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_serial_node = motor_interface_pkg.motor_serial_node:main',
        ],
    },
)
