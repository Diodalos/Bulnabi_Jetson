import os
from glob import glob
from setuptools import setup

package_name = 'px4_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ksj',
    maintainer_email='diodalos@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_broadcaster = px4_navigation.broadcast:main',
            'points_subscriber = px4_navigation.pointcloud:main'
        ],
    },
)
