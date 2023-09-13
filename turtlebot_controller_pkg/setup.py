from setuptools import find_packages, setup

package_name = 'turtlebot_controller_pkg'

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
    maintainer='Jonathan Trevatt',
    maintainer_email='jonathan@trevatt.net',
    description='This package controls a turtlebot3 robot. It should subscribe to the map and sensors created by the robot (or a gazebo simulation of the robot), and seeks to explore unexplored regions of the environment.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot_brain = turtlebot_controller_pkg.turtlebot_brain:main'
        ],
    },
)
