from setuptools import setup

package_name = 'bms_reader'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Install package.xml into share/central_comm
        ('share/' + package_name, ['package.xml']),
        # ament index resource (keeps ROS aware of this Python package)
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Max',
    maintainer_email='max.eberhard@gmx.net',
    description='ROS2 serial‐communication node for XiaoESP32 devices',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # This creates a "ros2 run central_comm bridge_node_node" executable
            'bms_node = bms_reader.bms_node:main',
        ],
    },
)