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
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

class Waypoint(PoseStamped):
    def __init__(self, x, y, orientation):
        super().__init__('waypoint')
        self.header.frame_id = 'map'
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.orientation.w = orientation

    def set_x(self, x):
        self.pose.position.x = x

    def set_y(self, y):
        self.pose.position.y = y

    def set_orientation(self, orientation):
        self.pose.orientation.w = orientation

    def get_x(self):
        return self.pose.position.x
    
    def get_y(self):
        return self.pose.position.y
    
    def get_orientation(self):
        return self.pose.orientation.w

class Brain(Node):
    def __init__(self):
        super().__init__('brain')
        # Publisher example code:
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.map = None
        # Subscriber example code:
        self.subscription = self.create_subscription(String,'topic',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

        ###TODO Subscribe to map
        # OccupancyGrid not in nav2_msgs/msg???
        self.map_subscription = self.create_subscription(OccupancyGrid,'map', self.map_callback, 10)
        
        self.status_subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_log_callback,
            10)
        
        self.waypoint_publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10)

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

    # map callback to assign map data to variables
    def map_callback(self, msg:OccupancyGrid):
        self.map = msg.data

    # If idle, calculate for another waypoint
    # from lab code
    def bt_log_callback(self, msg:BehaviorTreeLog):
        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery' and \
                event.current_status == 'IDLE':
                self.waypoint_compute(map)

    # TODO - Detect and react when navigation fails to find a valid path
    # TODO - Implement strategy for not re-sending bad waypoints
    def on_exploration_fail(self):
        pass

    # TODO - Detect unexplored areas of map
    def map_find_unexplored(self, map):
        pass

    # TODO - Generate a trivial waypoint (e.g., step forwards small amount)
    # To be used for checklist points in event non-trivial solution fails
    def waypoint_compute_trivial(self):
        pass

    # TODO - Implement exploration strategy to generate a test waypoint (based on map)
    def waypoint_compute(self, map):
        unexplored = self.map_find_unexplored(map) # must navigate robot to unexplored areas
        waypoint = None
        return waypoint
    
    # TODO - Check if a waypoint is reachable
    def waypoint_check_reachable(self, waypoint):
        return True

    def move_to_waypoint(self, waypoint):
        #Use nav2 or custom planning algorithm to move robot to waypoint
        if self.waypoint_check_reachable(waypoint):
            self.waypoint_publisher.publish(waypoint)
            PassFail = True
        PassFail = False
        return PassFail
    
    # TODO - Check if the robot has finished exploring the area
    def area_is_explored(self, map):
        isExplored = None
        return isExplored


def main(args=None):
    print('Hi from turtlebot_controller_pkg.')

    rclpy.init(args=args)
    brain = Brain()
    rclpy.spin(brain)
    brain.destroy_node() # Destroy the node explicitly
    rclpy.shutdown()

if __name__ == '__main__':
    main()
