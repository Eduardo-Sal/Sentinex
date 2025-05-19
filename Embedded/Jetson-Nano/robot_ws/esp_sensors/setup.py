from setuptools import find_packages, setup

package_name = 'esp_sensors'

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
    maintainer='asmr',
    maintainer_email='jasoyahir2003@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sound = esp_sensors.sound_sensor: main',
            'pairing = esp_sensors.pairing_button: main'
        ],
    },
)
