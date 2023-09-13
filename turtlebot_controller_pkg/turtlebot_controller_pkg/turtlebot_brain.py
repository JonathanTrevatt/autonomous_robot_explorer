"""
System Demonstration Checklist:
□ Automatically sends waypoints to the robot
    # Waypoints are automatically generated
    # Waypoints are manually generated
□ Uses SLAM to create a map
□ Algorithm for detecting unexplored areas
    # Finds unexplored areas
    # Finds unexplored areas next to known and free cells
□ Uses documented exploration strategy
    □ Subscribes to map
    □ Computes waypoint based on map
    □ Computes waypoint that is known to be reachable
□ Strategy works in all world files (entire space is visited)
    # Some areas not detected by algorithm
    # Entire space visited
    # All unexplored areas detected by algorithm but some ignored according to exploration strategy
□ Includes mechanism for detecting when exploration strategy fails
    □ Subscribes to navigation status
    □ Detects and reacts when navigation fails to find a valid path
    □ Strategy implemented for not re-sending bad waypoints
□ Algorithm for navigating robot to unexplored areas
    # Uses nav2 to move robot to waypoint
    # Uses custom planning algorithm to move robot to waypoint
□ Not hard-coded for certain test environments

Use slam to create map
subscribe to map
subscribe to navigation status
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


def main(args=None):
    print('Hi from turtlebot_controller_pkg.')
    rclpy.init(args=args)
    brain = Brain()
    rclpy.spin(brain)
    brain.destroy_node() # Destroy the node explicitly
    rclpy.shutdown()

if __name__ == '__main__':
    main()
