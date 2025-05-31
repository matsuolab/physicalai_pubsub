from setuptools import find_packages, setup

package_name = 'physicalai_pubsub'

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
    maintainer='Tatsuya Kamijo',
    maintainer_email='tatsukamijo@ieee.org',
    description='ROS2 example pub/sub package for Physical AI class at Matsuo-Iwasawa Lab, the University of Tokyo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_subscriber = physicalai_pubsub.state_subscriber:main',
            'action_publisher = physicalai_pubsub.action_publisher:main',
            'keyboard_teleop = physicalai_pubsub.keyboard_teleop:main',
        ],
    },
)
