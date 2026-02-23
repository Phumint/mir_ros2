import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'mir_driver_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add Launch
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phumint',
    maintainer_email='phumint1969@gmail.com',
    description='ROS2 Bridge for the MiR100',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mir_bridge = mir_driver_bridge.mir_bridge:main',
            'fake_mir_joint_publisher = mir_driver_bridge.fake_mir_joint_publisher:main',
            'tf_remove_child_frames = mir_driver_bridge.tf_remove_child_frames:main',
            'rep117_filter = mir_driver_bridge.rep117_filter:main',
        ],
    },
)