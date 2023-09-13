"""
System Demonstration Checklist:
□ Uses SLAM to create a map (I.e. launch file opens slam, this node subscribes to map)

Use slam to create map
subscribe to map
Detect unexplored area
    Suggestion: use watershedding method
    Fill region (with model of robot) from robot location (going around obstacles)
    explore first unexplored region that is filled
    I.e., goes to the closest unexplored area
generate suggested waypoint based on unexplored area of map
test if waypoint is reachable (if not, list it as a bad waypoint and pick a new one)
Go to waypoint (detect if navigation fails)
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Brain(Node):
    def __init__(self):
        super().__init__('brain')
        # Publisher example code:
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        # Subscriber example code:
        self.subscription = self.create_subscription(String,'topic',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

        ###TODO Subscribe to map
        ###TODO Subscribe to navigation status


    # timer_callback for publisher example code
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

    # listener_callback function for subscriber example code
    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

    # TODO - Detect and react when navigation fails to find a valid path
    # TODO - Implement strategy for not re-sending bad waypoints
    def on_exploration_fail():
        pass

    # TODO - Detect unexplored areas of map
    def map_find_unexplored(map):
        pass

    # TODO - Generate a trivial waypoint (e.g., step forwards small amount)
    # To be used for checklist points in event non-trivial solution fails
    def waypoint_compute_trivial():
        pass

    # TODO - Implement exploration strategy to generate a test waypoint (based on map)
    def waypoint_compute(map):
        unexplored = map_find_unexplored(map) # must navigate robot to unexplored areas
        waypoint = None
        return waypoint
    
    # TODO - Check if a waypoint is reachable
    def waypoint_check_reachable(waypoint):
        pass

    # TODO
    def move_to_waypoint(waypoint):
        #Use nav2 or custom planning algorithm to move robot to waypoint
        PassFail = None
        return PassFail


def main(args=None):
    print('Hi from turtlebot_controller_pkg.')

    rclpy.init(args=args)
    brain = Brain()
    rclpy.spin(brain)
    brain.destroy_node() # Destroy the node explicitly
    rclpy.shutdown()

if __name__ == '__main__':
    main()
