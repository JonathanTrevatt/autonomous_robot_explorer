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

from nav2_simple_commander.robot_navigator import BasicNavigator
from std_msgs.msg import String
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class Waypoint(PoseStamped):
    def __init__(self, x, y, z, orientation):
        super().__init__('waypoint')
        self.header.frame_id = 'map'
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = z
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
        self.cur_pos = Waypoint(0, 0, 0, 1)
        # Subscriber example code:
        self.subscription = self.create_subscription(String,'topic',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

        self.map_subscription = self.create_subscription(OccupancyGrid,'map', self.map_callback, 10)
        
        self.status_subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.bt_log_callback,
            10)
        
        self.position_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        self.waypoint_publisher = self.create_publisher(
            PoseStamped,
            'goal_pose',
            10)
        
        # Initialise navigator
        self.nav = BasicNavigator()
        init_pose = self.cur_pos
        self.nav.setInitialPose(init_pose)
        self.nav.waitUntilNav2Active()
        
    # USING NAV2 FOR AUTOMATIC PATH PLANNING

    def odom_callback(self, msg:Odometry):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_z = msg.pose.pose.position.z
        self.pos_w = msg.pose.pose.orientation.w
        self.cur_pos = Waypoint(
            self.pos_x,
            self.pos_y,
            self.pos_z,
            self.pos_w
        )

    #TODO Subscribe to error for unreachable path (in planner_server node)

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
                waypoint = self.waypoint_compute(map)
                self.move_to_waypoint(waypoint)

    # TODO - Detect and react when navigation fails to find a valid path
    # TODO - Implement strategy for not re-sending bad waypoints
    def on_exploration_fail(self):
        pass

    # TODO - Detect unexplored areas of map
    def map_find_unexplored(self, map):
        pass

    # TODO - Test
    # To be used for checklist points in event non-trivial solution fails
    # Returns a trivial waypoint set to move the robot a small distance from the current position
    def waypoint_compute_trivial(self, delta):
        x = self.pos_x + delta
        y = self.pos_y + delta
        z = self.pos_z
        w = self.pos_w
        waypoint = Waypoint(x, y, z, w)
        return waypoint


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
        #This requires sending initial pose and a first waypoint through command line
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
