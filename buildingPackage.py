"""
Navigate to src of the workspace (src is in workspace folder)

in console:
ros2 pkg create --build-type ament_python <package name>

Change maintainer, maintainer email, licence and description in both package.xml
and setup.py within the package

Create your python files within the created folders
MAKE SURE TO ADD DEPENDENCIES IN package.xml (e.g. strings require dependencies)
(e.g. <exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>)

In package.xml add the functions to run when the node is run
e.g.
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},

When completed, build your package:
colcon build --packages-select <my_package>

In another terminal, navigate to your workspace and source:
source install/local_setup.bash
"""