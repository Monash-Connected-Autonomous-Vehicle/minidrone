from setuptools import setup

package_name = 'motor_enc_interface'

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
    maintainer_email='s.a.baaset.moslih@gmail.com',
    description='Provides an interface for controlling mindrone motors and reading data from them',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_reader = motor_enc_interface.encoder_reader:main'
            'speed_to_serial = motor_enc_interface.espeed_to_serial:main'
        ],
    },
)
