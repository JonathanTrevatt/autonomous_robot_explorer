"""
Nodes: Single modular purpose module, sending or receiving data
Topics: Bus for nodes to exchange information
Service: Provides data when called upon
Action: Uses goal, feedback and result
Launch file: runs multiple nodes in one terminal

ROS2 console commands

ros2 pkg executables <node>
# show executable nodes in a package

ros2 run <package> <node>
# run a node in a package

rqt
# run rqt: calls services, 

rqt_graph
# Shows nodes, topics, connections, etc

ros2 run <package> <node> --ros-args --remap <entity/variable>:=<entity/variable>
# remap keys from one element to another

ros2 node list
# show all running nodes

ros2 node info </entity>

ros2 topic echo <entity/variable>
# shows information being published by the topic

ros2 topic pub (--once) <topic_name> <msg_type> '<args>'
# publish information directly to the topic

ros2 topic hz <topic>
# shows topic publish refresh rate

ros2 service type <service>
# returns data types sent and received

ros2 service find <type_name>
# finds all services of type

ros2 service call <service_name> <service_type> <arguments>
# calls upon a service

ros2 param list
# finds all parameters

ros2 param get <node_name> <parameter_name>
# returns value for a parameter

ros2 param set <node_name> <parameter_name> <value>

ros2 action send_goal <action_name> <action_type> <values>

ros2 run rqt_console rqt_console
# displays error messages

ros2 launch <package> <launch_file>
# run a launch file

ros2 bag record <topic_name>
# begin recording commands to a topic

ros2 bag record -o <new_file_name> <topic1> <topic2> ...
# record from multiple topics into a file

ros2 bag play subset
# play back topic data from file

ros2 topic echo /turtle1/cmd_vel
"""