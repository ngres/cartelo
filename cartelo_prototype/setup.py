from os import path
from glob import glob
from setuptools import find_packages, setup

package_name = 'cartelo_prototype'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (path.join('share', package_name, 'config'), glob('config/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='nicolas@gres.io',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gripper = cartelo_prototype.gripper:main',
            'pose = cartelo_prototype.pose:main'
        ],
    },
)
