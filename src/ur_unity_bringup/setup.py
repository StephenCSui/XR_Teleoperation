import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'ur_unity_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steph',
    maintainer_email='stephenchristian.suiwinata@student.uts.edu.au',
    description='UR + Unity bringup nodes',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'startup_pose_and_servo_bootstrap = ur_unity_bringup.startup_pose_and_servo_bootstrap:main',
            'unity_pose_to_servo_twist = ur_unity_bringup.unity_pose_to_servo_twist:main',
            'ee_tf_to_pose = ur_unity_bringup.ee_tf_to_pose:main',
            'pose_error_to_twist = ur_unity_bringup.pose_error_to_twist:main',
            'servo_auto_start = ur_unity_bringup.servo_auto_start:main',
        ],
    },
)