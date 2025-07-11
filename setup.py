from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'multiview_saver'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch/*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuhsienc',
    maintainer_email='illusion.dark@gmail.com',
    description='This ROS2 node subscribes data from a RGBD camera and a robot arm periodically, and saves data to files if request received.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'save_three_view = multiview_saver.save_three_view:main'
        ],
    },
)
