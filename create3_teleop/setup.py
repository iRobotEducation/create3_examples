import os
from glob import glob
from setuptools import setup

package_name = 'create3_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        (os.path.join('share','ament_index', 'resource_index', 'packages'),
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name),
            ['package.xml']),
        (os.path.join('share', package_name, "launch"),
            glob(os.path.join('launch', '*launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alberto Soragna',
    maintainer_email='asoragna@irobot.com',
    description='Example launch files for teleoperating the iRobot(R) Create(R) 3 Educational Robot.',
    license='BSD-3',
    tests_require=['pytest'],
)
