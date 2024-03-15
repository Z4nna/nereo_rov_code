from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'nereo_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michele Carenini',
    maintainer_email='michele.carenini@studenti.polito.it',
    description='All the ROS files needed to run Nereo ROV',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "camera1_pub = nereo_pkg.camera1_pub:main",
            "control_station = nereo_pkg.gui:main",
        ],
    },
)
