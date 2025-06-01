from setuptools import setup

package_name = 'serial_comm'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Install package.xml into share/serial_comm
        ('share/' + package_name, ['package.xml']),
        # ament index resource (keeps ROS aware of this Python package)
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=[
        'setuptools',
        'pyserial'         # for SerialProcessor to work
    ],
    zip_safe=True,
    maintainer='Max',
    maintainer_email='max.eberhard@gmx.net',
    description='ROS2 serial‐communication node for Arduino devices',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # This creates a "ros2 run serial_comm serial_node" executable
            'serial_node = serial_comm.serial_node:main',
        ],
    },
)