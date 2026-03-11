from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'intelligent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Intelligent state machine',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'intelligent_node = intelligent.state_machine:main',
        ],
    },
)