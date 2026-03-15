from setuptools import setup

package_name = 'tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tracking.launch.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='Max',
    maintainer_email='max.eberhard@gmx.net',
    description='ROS2 multi-object tracking node with Kalman-filter based tracking',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tracking_node = tracking.tracking_node:main',
        ],
    },
)