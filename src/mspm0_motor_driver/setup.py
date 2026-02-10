from setuptools import find_packages, setup

package_name = 'mspm0_motor_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/mspm0_motor_driver']),
        ('share/mspm0_motor_driver', ['package.xml']),
        ('share/mspm0_motor_driver/launch', ['launch/motor.launch.py']),
        ('share/mspm0_motor_driver/config', ['config/params.yaml']),
    ],
    install_requires=[
        'setuptools',
        'pyserial',
    ],
    zip_safe=True,
    maintainer='regiroy',
    maintainer_email='regiroy@todo.todo',
    description='ROS 2 driver for MSPM0-based 4-channel motor controller',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mspm0_motor_driver = mspm0_motor_driver.motor_driver_node:main',
        ],
    },

)
