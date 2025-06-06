from setuptools import find_packages, setup

package_name = 'sensor_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jhsrobo',
    maintainer_email='jammerand14@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_sensor = sensor_interface.depth_sensor:main',
            'imu_sensor = sensor_interface.imu_sensor:main',
            'inner_temp_sensor = sensor_interface.inner_temp_sensor:main',
            'leak_sensor = sensor_interface.leak_sensor:main',
            'outer_temp_sensor = sensor_interface.outer_temp_sensor:main',
            'electronics_sensor = sensor_interface.electronics_sensor:main',
            'ph_sensor = sensor_interface.ph_sensor:main'
        ],
    },
)
