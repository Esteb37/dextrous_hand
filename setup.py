from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dextrous_hand'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join("launch", '*.[py][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='esteb37',
    maintainer_email='esteban37padilla@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_node = src.hand_node:main',
            'basic_node = src.basic_node:main',
            'keyboard_node = src.keyboard_node:main',
            'mujoco_node = src.mujoco_node:main',
            'calibrate_node = src.calibrate_node:main',
            'webcam_mano_node = src.webcam_mano_node:main',
            'retargeter_node = src.retargeter_node:main',
            'visualize_joints_node = src.visualize_joints_node:main',
            'rokoko_node = src.rokoko_node:main',
            'arm_node = src.arm_node:main',
        ],
    },
)
