from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'motor_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), 
 glob('launch/*.py')),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='puja',
    maintainer_email='puja@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_drive_node = motor_driver.serial_drive_node:main',
            'teleop_gui_node = motor_driver.teleop_gui_node:main',
            'odom_monitor_node = motor_driver.odom_monitor_node:main',
        ],
    },
)
