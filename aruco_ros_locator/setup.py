import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'aruco_ros_locator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='david@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "locator = aruco_ros_locator.locator:main",
            "accuracy_meas = aruco_ros_locator.accuracy_meas:main",
            "static_tf_broadcast = aruco_ros_locator.static_tf_broadcast:main"
        ],
    },
)
