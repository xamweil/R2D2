from setuptools import setup

package_name = 'scene_understanding'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/scene_understanding.launch.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Max',
    maintainer_email='max.eberhard@gmx.net',
    description='ROS2 TensorRT-based visual scene understanding node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scene_understanding_node = scene_understanding.scene_understanding_node:main',
        ],
    },
)