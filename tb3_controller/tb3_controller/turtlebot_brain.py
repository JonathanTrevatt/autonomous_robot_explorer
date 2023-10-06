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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import OccupancyGrid, Odometry
import os
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class Brain(Node):
    def __init__(self):
        super().__init__('brain')

        # Publisher example code:
        timer_period = 0.5  # seconds
        self.i = 0
        self.map = None

        print('turtlebot_brain.Brain: instantiating subscriptions')
        # Subscriber example code:
        self.map_subscription       = self.create_subscription  (OccupancyGrid,             'map',                  self.map_callback,      10)
        self.status_subscription    = self.create_subscription  (BehaviorTreeLog,           'behavior_tree_log',    self.bt_log_callback,   10)
        self.position_subscription  = self.create_subscription  (Odometry,                  'odom',                 self.odom_callback,     10)
        self.waypoint_publisher     = self.create_publisher     (PoseStamped,               'goal_pose',    10)
        
        print("turtlebot_brain.Brain: defining qos_profile")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.SYSTEM_DEFAULT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)

        print("turtlebot_brain.Brain: creating publishers")
        self.amcl_pose_publisher    = self.create_publisher     (PoseWithCovarianceStamped, 'amcl_pose',    qos_profile=qos_profile)
        self.init_pose_publisher    = self.create_publisher     (PoseWithCovarianceStamped, 'initialpose',  10)
        
        print("turtlebot_brain.Brain: Initialising navigator")
        self.first = True
        self.nav = BasicNavigator() # Initialise navigator
        self.nav.lifecycleStartup() #init_pose = self.cur_pos

    # USING NAV2 FOR AUTOMATIC PATH PLANNING

    # DEFINING CALLBACK FUNCTIONS
    def odom_callback(self, msg:Odometry):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_w = msg.pose.pose.orientation.w
        self.cur_pos = PoseStamped()
        self.cur_pos.pose.position.x = self.pos_x
        self.cur_pos.pose.position.y = self.pos_y
        self.cur_pos.pose.orientation.w = self.pos_w
        if self.first:  
            self.first = False
            self.move_to_waypoint(1.0, -0.5, 1)
            

    #TODO Subscribe to error for unreachable path (in planner_server node)

    # timer_callback for publisher example code

    # listener_callback function for subscriber example code

    # map callback to assign map data to variables
    def map_callback(self, msg:OccupancyGrid):
        self.map = msg.data

    # If idle, calculate for another waypoint from lab code
    def bt_log_callback(self, msg:BehaviorTreeLog):
        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery' and \
                event.current_status == 'IDLE':
                waypoint = self.waypoint_compute(map)
                self.move_to_waypoint(1.0, -0.5, 1)

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

    # TODO - Low priority - Modify to use python functions instead of just passing a string command directly to terminal.
    def move_to_waypoint(self, x, y, w):
        #Use nav2 or custom planning algorithm to move robot to waypoint
        #This requires sending initial pose and a first waypoint through command line
        print('turtlebot_brain.move_to_waypoint: Setting waypoint {position: {x: %s, y: %s}, orientation: {w: %s}}'  % (x, y, w))
        os.system("ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped '{header: {frame_id: 'map'}, pose: {position: {x: %s, y: %s}, orientation: {w: %s}}}'" % (x, y, w))
    
    # TODO - Check if the robot has finished exploring the area
    def area_is_explored(self, map):
        isExplored = None
        return isExplored


def main(args=None):
    print('turtlebot_brain.main: Starting main')
    print('turtlebot_brain.main: instantiating rclpy')
    rclpy.init(args=args)
    print('turtlebot_brain.main: instantiating brain')
    brain = Brain()
    print('turtlebot_brain.main: spinning brain')
    rclpy.spin(brain)
    print('turtlebot_brain.main: destroying brain')
    brain.destroy_node() # Destroy the node explicitly
    print('turtlebot_brain.main: shutting down rclpy')
    rclpy.shutdown()

if __name__ == '__main__':
    main()