from setuptools import setup
from glob import glob

package_name = 'md_lane_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='byron',
    maintainer_email='bgel0002@student.monash.edu',
    description='lane detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sense_node = md_lane_detection.sense_node:main',
            'follow_node = md_lane_detection.follow_node:main'
        ],
    },
)
