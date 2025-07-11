from setuptools import find_packages, setup

package_name = 'ros_mqtt_bridge'

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
    maintainer='zxro',
    maintainer_email='duddms5239@hanyang.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'ros_to_mqtt_bridge = ros_mqtt_bridge.ros_to_mqtt_bridge:main',
    ],
},
)
