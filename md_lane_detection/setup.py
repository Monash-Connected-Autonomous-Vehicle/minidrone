from setuptools import setup

package_name = 'md_lane_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'sense = md_lane_detection.sense_node:main',
            'follow = md_lane_detection.follow_node:main'
        ],
    },
)
