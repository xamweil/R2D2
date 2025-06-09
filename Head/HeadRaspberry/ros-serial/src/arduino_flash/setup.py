from setuptools import setup

package_name = 'arduino_flash'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Install package.xml into share/arduino_flash
        ('share/' + package_name, ['package.xml']),
        # ament index resource (keeps ROS aware of this Python package)
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=[
        'setuptools',
        'serial'  
    ],  
    zip_safe=True,
    maintainer='Max',
    maintainer_email='max.eberhard@gmx.net',
    description='ROS2 Arduino flashing node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # This creates a "ros2 run arduino_flash flash_node" executable
            'flash_node = arduino_flash.flash_node:main',
        ],
    },
)