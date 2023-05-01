import os
from setuptools import setup
from glob import glob

package_name = 'delivery'

setup(
    name=package_name,
    version='0.0.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bill Mania',
    maintainer_email='bill@manialabs.us',
    description='ROS2 node to manage seatrac ACOMM/USBL X150 and X110 devices.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'delivery = delivery.delivery_node:main'
        ],
    },
)
