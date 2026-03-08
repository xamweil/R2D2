from setuptools import setup, find_packages

package_name = 'ui_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Max',
    maintainer_email='max.eberhard@gmx.net',
    description='ROS2 UI bridge node for web interface communication',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'api_node = ui_bridge.api_node:main',
        ],
    },
)