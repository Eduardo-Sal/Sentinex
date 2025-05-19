from setuptools import find_packages, setup

package_name = 'esp32_controller'

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
    maintainer_email='asmr@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mqtt_control = esp32_controller.mqtt_control: main",
            "connection_helper = esp32_controller.connection_helper: main",
            "motor_control_Twist = esp32_controller.motor_control_Twist: main",
            "joint_state_publisher = esp32_controller.joint_state_publisher:main"
        ],
    },
)
