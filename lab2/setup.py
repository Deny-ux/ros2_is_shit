from setuptools import setup
import os
from glob import glob

package_name = 'lab2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/lab2.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siwy',
    maintainer_email='01168879@pw.edu.pl',
    description='Package to steer turtles',
    license='License: None',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "turtle_spawner = lab2.turtles_spawner:main"
        ],
    },
)
