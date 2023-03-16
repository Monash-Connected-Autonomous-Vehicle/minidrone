from setuptools import setup

package_name = 'piksi_gps'

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
    maintainer='mcav',
    maintainer_email='monashcav@gmail.com',
    description='Read GPS data from Piksi via a serial connection using the sbp python library.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'gps = piksi_gps.piksi_gps_pub:main',
        ],
    },
)
