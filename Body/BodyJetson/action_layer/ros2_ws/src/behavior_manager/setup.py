from setuptools import setup

package_name = 'behavior_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=[
        package_name,
        package_name + '.skills',
    ],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/behavior_manager.launch.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Max',
    maintainer_email='max.eberhard@gmx.net',
    description='ROS2 behavior manager node for action execution and skill orchestration',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'behavior_manager_node = behavior_manager.behavior_manager_node:main',
        ],
    },
)