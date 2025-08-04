from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'cobot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/doc', glob('doc/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takashi Otsuka',
    maintainer_email='takatronix@gmail.com',
    description='myCobot280 Enhanced Control Package - Simple name, powerful features',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cobot_node = cobot.cobot_node:main',
        ],
    },
)
