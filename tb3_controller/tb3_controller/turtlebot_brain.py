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
import numpy as np
import random

class Brain(Node):
    def __init__(self):
        super().__init__('brain')
        self.map = None
        self.unreachable_positions = []

        print('NOTE - turtlebot_brain.Brain: instantiating subscriptions')
        # Subscriber example code:
        self.position_subscription  = self.create_subscription  (Odometry,                  'odom',                 self.odom_callback,     10)
        self.map_subscription       = self.create_subscription  (OccupancyGrid,             'map',                  self.map_callback,      10)
        self.status_subscription    = self.create_subscription  (BehaviorTreeLog,           'behavior_tree_log',    self.bt_log_callback,   10)
        self.waypoint_publisher     = self.create_publisher     (PoseStamped,               'goal_pose',    10)
        
        print("NOTE - turtlebot_brain.Brain: defining qos_profile")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.SYSTEM_DEFAULT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)


        print("NOTE - turtlebot_brain.Brain: Initialising navigator")
        self.first = True
        self.ready_to_move = True
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
    
    """
    Converts x,y map pixel coords to global coords in m.
    """
    def coord_mapPxl2m(self, mapPos_x, mapPos_y):
        pos_x = (mapPos_x * self.mapInfo.resolution) + self.mapInfo.origin.position.x
        pos_y = (mapPos_y * self.mapInfo.resolution) + self.mapInfo.origin.position.y
        return pos_x, pos_y
    
    """
    Converts global x,y coords in m to pixel coords on the map.
    """
    def coord_m2mapPxl(self, pos_x, pos_y):
        mapPos_x = int((pos_x - self.mapInfo.origin.position.x)/self.mapInfo.resolution)
        mapPos_y = int((pos_y - self.mapInfo.origin.position.y)/self.mapInfo.resolution)
        return mapPos_x, mapPos_y
    
    """
    Returns the current position as the x,y pixel position on the map.
    """
    def get_coords_asMapPxl(self):
        return self.coord_m2mapPxl(self.pos_x, self.pos_y)
    
    """
    Marks a map x,y pixel position as unreachable so that we won't try to pathfind there in future.
    """
    def mark_mapPxl_unreachable(self, x, y):
        x = int(x)
        y = int(y)
        for i in range(0, 5):
            for j in range(0, 5):
                self.unreachable_positions.append([x + i, y + j])
        return
    
    """
    Returns a bool: true if given x,y map pixel coords have been previously marked as unreachable - else false.
    """
    def is_mapPxl_unreachable(self, x, y):
        if [x,y] in self.unreachable_positions:
            return True
        return False

    """
    Attempts to generate a path to a waypoint. On failure, returns False, else, True.
    """
    def try_generate_path(self, x, y):
        #waypoint = PoseStamped()
        #waypoint.pose.position.x = float(x)
        #waypoint.pose.position.y = float(y)
        #waypoint.pose.orientation.w = 0.0
        #path = self.nav.getPath(self.cur_pos, waypoint)
        #if path != None:
            return True
        #return False

    #TODO Subscribe to error for unreachable path (in planner_server node)

    # timer_callback for publisher example code

    # listener_callback function for subscriber example code

    
    """
    map callback to assign map data to variables
    Represents a 2-D grid map, in which each cell represents the 
    probability of occupancy.
    Values range [-1, 100], where -1 represents an unknown probablility.
    """
    def map_callback(self, msg:OccupancyGrid):
        print('NOTE - turtlebot_brain.map_callback: reached')
        self.mapArray2d = np.reshape(msg.data, (-1, msg.info.width))
        self.mapInfo = msg.info

    # If idle, calculate for another waypoint from lab code
    def bt_log_callback(self, msg:BehaviorTreeLog):
        print('NOTE - turtlebot_brain.bt_log_callback: reached')
        for event in msg.event_log:
            if event.node_name == 'NavigateRecovery' and \
                event.current_status == 'IDLE':
                self.domap(self.mapInfo)

    # TODO - Detect and react when navigation fails to find a valid path
    # TODO - Implement strategy for not re-sending bad waypoints
    def on_exploration_fail(self):
        print('NOTE - turtlebot_brain.on_exploration_fail: reached')
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
        print('NOTE - turtlebot_brain.waypoint_compute: reached')
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
        waypoint = PoseStamped()
        waypoint.pose.position.x = float(x)
        waypoint.pose.position.y = float(y)
        waypoint.pose.orientation.w = float(w)
        print('NOTE - turtlebot_brain.move_to_waypoint: Setting waypoint {position: {x: %s, y: %s}, orientation: {w: %s}}'  % (x, y, w))
        os.prr
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()

    def map_get_unexplored_in_range(self, mapx, mapy, radius):
        print("Getting unexplored pixels in range.")
        unexplored_in_range = []
        xmin = max(mapx-radius, 0)
        print(mapx, mapy, radius)
        xmax = min(mapx+radius, self.mapInfo.width)
        ymin = max(mapy-radius, 0)
        ymax = min(mapy+radius, self.mapInfo.height)

        print("xmin, xmax, ymin, ymax: ", xmin, xmax, ymin, ymax)
        print("mapArray2d.shape:", self.mapArray2d.shape)
        for mapx in np.linspace(xmin, xmax, xmax-xmin):
            for mapy in np.linspace(ymin, ymax, ymax-ymin):
                mapx=int(mapx)
                mapy=int(mapy)
                if self.mapArray2d[mapx][mapy] == -1: # if pixel is unexplored
                    unexplored_in_range.append([mapx,mapy])
        return unexplored_in_range
        
    def test_unexplored(self, unexplored_list):
        print("test_unexplored - Testing unexplored map pixels from unexplored_list.")
        print("test_unexplored - unexplored_list: ", unexplored_list)
        for i in range(len(unexplored_list)):
            mapx, mapy = unexplored_list.pop() 
            if not self.is_mapPxl_unreachable(mapx,mapy):   # if pixel is possibly reachable
                x, y = self.coord_mapPxl2m(mapx, mapy)
                print(x, y)
                if self.try_generate_path(x, y):    # and attempt to generate a path succeeds
                    self.move_to_waypoint(x, y, 0) # and waypoint is reached (within time limit)
                    result = self.nav.getResult() 
                    if result == result.SUCCEEDED:
                        return True # job done
                # If we find pixel to be unreachable
                self.mark_mapPxl_unreachable(mapx,mapy)
            # If randomly selected pixel is unreachable
            # try another pixel from list
        # If every element of the list is unreachable
        return False

    def domap(self, msg:OccupancyGrid):
        """
        self.pos_x:  (position of robot in global coords in m)
        map resolution: each pixel of the map represents 0.05m
        msg.info.origin.position.x: position of lower left pixel of map (origin) in global coords in m
        # TODO - implement try_generate_path
        # TODO - We currently need to wait for move_to_waypoint to complete before marking it as a 
        success or failure and trying a new pixel. We also currently wait within this function, 
        which would stop all operations while we wait for movement to complete. We need to fix this.
        # TODO - ensure we don't generate path twice when we call try_generate_path and 
        move_to_waypoint (possibly by joining them together as a single function)
        """
        mapx, mapy = self.get_coords_asMapPxl()
        min_search_radius = 10
        max_search_radius = 50
        search_radius = min_search_radius
        # search radius for reachable, unexplored pixels and set goal to go there
        while search_radius < max_search_radius:
            print("domap - Searching for unexplored pixels in radius: ", search_radius)
            # generate list of unexplored pixels within search radius
            unexplored_list = self.map_get_unexplored_in_range(mapx, mapy, search_radius)
            print("domap - unexplored_list: ", unexplored_list)
            if unexplored_list is not None:
                print("domap - unexplored_list is not None - unexplored_list = \n", unexplored_list)
                print("randomly shuffling unexplored_list to remove preference for exploring in a certain direction.")
                random.shuffle(unexplored_list)
                print("Shuffled list = \n", unexplored_list)
                print("domap - calling self.test_unexplored(unexplored_list)")
                if self.test_unexplored(unexplored_list):
                    print("domap - List element unreachable or is already known")
                    return True # Stop searching
            print("domap - unexplored_list is None, or every unexplored element is unreachable.")
            print("Expanding search radius.")
            search_radius += 5 # Expand search radius
        print("domap - Maximum search radius is reached.")
        print("domap - Stopping.")
        return False

        # TODO
        # check each pxl within range of robot for unexplored pxl
        # for first unexplored pxl found: 
        #   check if a path to it can be found
        #   if a path can be found, set waypoint
        #   wait of  goal to be reached (or a set delay)
        # repeat process
        # if no reachable unexplored pxls in range, increase range and repeat




def main(args=None):
    print('NOTE - turtlebot_brain.main: Starting main')
    print('NOTE - turtlebot_brain.main: instantiating rclpy')
    rclpy.init(args=args)
    print('NOTE - turtlebot_brain.main: instantiating brain')
    brain = Brain()

    print('NOTE - turtlebot_brain.main: spinning brain')
    rclpy.spin(brain)
    print('NOTE - turtlebot_brain.main: destroying brain')
    brain.destroy_node() # Destroy the node explicitly
    print('NOTE - turtlebot_brain.main: shutting down rclpy')
    rclpy.shutdown()

if __name__ == '__main__':
    main()