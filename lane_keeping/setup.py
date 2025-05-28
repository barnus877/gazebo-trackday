from setuptools import find_packages, setup

package_name = 'lane_keeping'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Barnabás Szabó',
    maintainer_email='barnus877&gmail.com',
    description='Python nodes for  Gazebo Harmonic and ROS Jazzy for trackday package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'image_republisher = bme_gazebo_sensors_py.image_republisher:main',
            'lane_keeping = lane_keeping.lane_keeping:main',
            #'chase_the_ball = bme_gazebo_sensors_py.chase_the_ball:main',
        ],
    },
)