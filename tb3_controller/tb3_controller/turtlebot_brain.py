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
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.msg import BehaviorTreeLog
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import os
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import random
from array import array

class Brain(Node):
    def __init__(self):
        super().__init__('brain')
        self.unreachable_positions = [] # set to np.zeros((msg.info.height, msg.info.width), dtype=bool) once in map callback
        # flags that determine if basic data has been instantiated by callbacks yet
        self.ready_odom = False
        self.ready_map = False
        self.ready_log = False
        self.first_waypoint_sent = False
        self.nav_canceled = False
        self.map_unreachable_initFlag = False

        print('NOTE - turtlebot_brain.Brain: instantiating subscriptions')
        # Subscriber example code:
        self.map_subscription       = self.create_subscription  (OccupancyGrid,             'map',                  self.map_callback,      10)
        self.status_subscription    = self.create_subscription  (BehaviorTreeLog,           'behavior_tree_log',    self.bt_log_callback,   10)
        self.position_subscription  = self.create_subscription  (Odometry,                  'odom',                 self.odom_callback,     10)
        self.path_subscription      = self.create_subscription  (Path,                      'local_plan',           self.path_callback,     10)
        self.waypoint_publisher     = self.create_publisher     (PoseStamped,               'goal_pose',    10)
        self.map_reachable_publisher= self.create_publisher     (OccupancyGrid,             'map_reachable',    10)

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.SYSTEM_DEFAULT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)

        self.nav = BasicNavigator() # Initialise navigator
        self.nav.lifecycleStartup() #init_pose = self.cur_pos

    # USING NAV2 FOR AUTOMATIC PATH PLANNING

    # DEFINING CALLBACK FUNCTIONS
    def odom_callback(self, msg:Odometry):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.pos_w = msg.pose.pose.orientation.w
        self.cur_pos = PoseStamped()
        self.cur_pos.header.frame_id = 'map'
        self.cur_pos.pose.position.x = self.pos_x
        self.cur_pos.pose.position.y = self.pos_y
        self.cur_pos.pose.orientation.w = self.pos_w
        if self.ready_map and not self.first_waypoint_sent:  
            self.first_waypoint_sent = True
            waypoint = (self.pos_x, self.pos_y, self.pos_w)
            self.move_to_waypoint(waypoint)
        self.ready_odom = True
        
    def map_callback(self, msg:OccupancyGrid):
        """
        map callback to assign map data to variables
        Represents a 2-D grid map, in which each cell represents the 
        probability of occupancy.
        Values range [-1, 100], where -1 represents an unknown probablility.
        """
        print('NOTE - turtlebot_brain.map_callback: reached')
        self.mapMsg = msg
        self.mapArray2d = np.reshape(msg.data, (msg.info.width,-1))
        self.mapInfo = msg.info
        if self.unreachable_positions == []:
            self.unreachable_positions = np.zeros((msg.info.width+1, msg.info.height+1), dtype=bool)
        
        if not self.map_unreachable_initFlag:
            self.init_map_unreachable(msg)
            self.map_reachable_publisher.publish(self.map_unreachable)

        self.ready_map = True
        self.map_reachable_publisher.publish(self.map_unreachable)
        return
    
    def path_callback(self, msg:Path):
        self.path = msg
    
    def init_map_unreachable(self, msg:OccupancyGrid):
        self.map_unreachable_initFlag = True

        self.map_unreachable = OccupancyGrid()
        self.map_unreachable.header.frame_id = "map"
        self.map_unreachable.info = msg.info
        size = np.array(msg.data).size
        map_unreachable_data = array('b', [int(xv) if c else 101*int(yv) for c, xv, yv in zip(np.array(msg.data) > 80, np.zeros(size), np.ones(size))])

        self.map_unreachable.data = map_unreachable_data
        return

    
    def bt_log_callback(self, msg:BehaviorTreeLog):
        for event in msg.event_log:
            if (event.node_name == 'NavigateRecovery' and \
                event.current_status == 'IDLE') or self.nav_canceled:
                self.nav_canceled = False
                if self.ready_odom and self.ready_map:
                    waypointPxl = self.waypointPxl_compute()
                    print("waypointPxl: ", waypointPxl)
                    waypoint = self.coord_pxl2m(waypointPxl)
                    print("waypoint: ", waypoint)
                    self.move_to_waypoint(waypoint)
            else: print("robot busy")
        self.ready_log = True

    def coord_pxl2m(self, waypointPxl):
        """
        Converts x,y,w map pixel coords to global coords in m.
        """
        mapPos_x, mapPos_y = waypointPxl
        pos_x = (mapPos_x * self.mapInfo.resolution) + self.mapInfo.origin.position.x
        pos_y = (mapPos_y * self.mapInfo.resolution) + self.mapInfo.origin.position.y
        pos_w = 0
        waypoint = (pos_x, pos_y, pos_w)
        return waypoint

    def coord_m2pxl(self, waypoint):
        """
        Converts global x,y coords in m to pixel coords on the map.
        """
        pos_x, pos_y, pos_w = waypoint
        mapPos_x = int((pos_x - self.mapInfo.origin.position.x)/self.mapInfo.resolution)
        mapPos_y = int((pos_y - self.mapInfo.origin.position.y)/self.mapInfo.resolution)
        waypointPxl = (mapPos_x, mapPos_y)
        return waypointPxl
    
    def get_coords_as_Pxl(self):
        """
        Returns the current position as the x,y pixel position on the map.
        """
        waypoint = (self.pos_x, self.pos_y, self.pos_w)
        return self.coord_m2pxl(waypoint)
    
    def mark_range_unreachable(self, pxl, radius):
        x_posPxl, y_posPxl = pxl
        xmin = max(x_posPxl-radius, 0)
        xmax = min(x_posPxl+radius, self.mapInfo.width - 1)
        ymin = max(y_posPxl-radius, 0)
        ymax = min(y_posPxl+radius, self.mapInfo.height - 1)

        print("mapArray2d.shape:", self.mapArray2d.shape)
        for xPxl in np.linspace(xmin, xmax, xmax-xmin):
            for yPxl in np.linspace(ymin, ymax, ymax-ymin):
                pxl = (int(xPxl), int(yPxl))
                self.mark_waypointPxl_unreachable(pxl)
        self.map_reachable_publisher.publish(self.map_unreachable)

    """
    def mark_waypointPxl_unreachable(self, waypointPxl):
        ""
        Marks a map x,y pixel position as unreachable so that we won't try to pathfind there in future.
        ""
        print("Marking waypoint as unreachable.")
        xPxl, yPxl = waypointPxl
        self.unreachable_positions[xPxl][yPxl] = True
        return
    """
    def mark_area_unreachable(self, waypointPxl):
        print("Marking waypoint as unreachable.")
        xPxl, yPxl = waypointPxl
        i = 0
        j = 0
        while self.mapArray2d[xPxl + i][yPxl] == -1 and xPxl + i < self.mapMsg.info.width:
            while self.mapArray2d[xPxl + i][yPxl + j] == -1 and yPxl + j < self.mapMsg.info.height:
                self.unreachable_positions[xPxl + i][yPxl + j] = True
                j += 1
            j = 0
            i += 1
        while self.mapArray2d[xPxl - i][yPxl] == -1 and xPxl - i >= 0:
            while self.mapArray2d[xPxl - i][yPxl - j] == -1 and yPxl + j >= 0:
                self.unreachable_positions[xPxl + i][yPxl + j] = True
                j += 1
            j = 0
            i += 1 
        self.unreachable_positions[xPxl][yPxl] = True
        return


    def is_waypointPxl_unreachable(self, waypointPxl):
        """
        Returns a bool: true if given x,y map pixel coords have been previously marked as unreachable - else false.
        """
        xPxl, yPxl = waypointPxl
        return self.unreachable_positions[xPxl][yPxl]

    # TODO - implement
    def try_generate_path(self, waypoint):
        """
        Attempts to generate a path to a waypoint. On failure, returns False, else, True.
        """

        new_pose = PoseStamped()
        new_pose.pose.position.x = float(waypoint[0])
        new_pose.pose.position.y = float(waypoint[1])
        new_pose.pose.orientation.w = float(waypoint[2])
        new_pose.header.frame_id = 'map'
        cur_pose = PoseStamped()
        cur_pose.pose.position.x = self.pos_x
        cur_pose.pose.position.y = self.pos_y
        cur_pose.pose.position.z = 0.0
        cur_pose.pose.orientation.x = 0.0
        cur_pose.pose.orientation.y = 0.0
        cur_pose.pose.orientation.z = 0.0
        cur_pose.pose.orientation.w = 0.0
        cur_pose.header.frame_id = 'map'
        path = self.nav.getPath(cur_pose, new_pose)
        if path is not None:
            return True
        return False
    
    
    """
    def waypoint_compute_trivial(self):
        waypoint = (0.5, 0.5, 1)
        return waypoint
    """

    def waypointPxl_compute(self):
        """
        Returns a goal waypoint to pathfind to unexplored areas of the map.
        """
        print('NOTE - turtlebot_brain.waypoint_compute: reached')
        xPxl, yPxl = self.get_coords_as_Pxl()
        min_search_radius = 10
        max_search_radius = 30
        search_radius = min_search_radius
        # search radius for reachable, unexplored pixels and set goal to go there
        while search_radius <= max_search_radius:
            print("waypoint_compute - Searching for unexplored pixels in radius: ", search_radius)
            # generate list of unexplored pixels within search radius
            unexplored_list = self.map_get_unexplored_in_range(search_radius)
            print("waypoint_compute - unexplored_list: ", unexplored_list)
            if unexplored_list is not None:
                print("waypoint_compute - unexplored_list is not None - unexplored_list = \n", unexplored_list)
                print("randomly shuffling unexplored_list to remove preference for exploring in a certain direction.")
                random.shuffle(unexplored_list)
                print("Shuffled list = \n", unexplored_list)
                print("waypoint_compute - calling self.waypoint_check_reachable(unexplored_list)")
                reachable_waypoint_pxl = self.waypoint_check_reachable(unexplored_list)
                if reachable_waypoint_pxl is not None:
                    return reachable_waypoint_pxl # Stop searching
            print("waypoint_compute - unexplored_list is None, or every unexplored element is unreachable.")
            print("Expanding search radius.")
            search_radius += 5 # Expand search radius
        print("waypoint_compute - Maximum search radius is reached.")
        print("waypoint_compute - Stopping.")
        return None # If no valid points are found, return None

    # TODO - Low priority - Use python function instead of terminal command to implement
    def move_to_waypoint(self, waypoint):
        """
        Publishes waypoint for nav2 to guide robot, using terminal commands.
        """
        x, y, w = waypoint
        print("Moving to waypoint: ", waypoint)
        self.IDLE = False
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.orientation.w = float(waypoint[2])
        self.nav.goToPose(pose)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=10.0):
                self.nav_canceled = True
                self.nav.cancelTask()
        result = self.nav.getResult()
        if result == result.CANCELED or result == result.FAILED:
            self.mark_range_unreachable(self.coord_m2pxl(waypoint), 10)
    
    def move_to_waypointPxl(self, waypointPxl):
        """
        Does the same as move_to_waypoint(self, waypoint), but accepts map pixel coordinates.
        """
        waypoint = self.coord_pxl2m(waypointPxl)
        self.move_to_waypoint(waypoint)

    def map_get_unexplored_in_range(self, radius):
        """
        Searches a range of map pixels and returns a list of unexplored pixels in that range.
        """
        current_posPxl = self.get_coords_as_Pxl()
        x_posPxl, y_posPxl = current_posPxl
        print("Getting unexplored pixels in range.")
        unexplored_in_range = []
        print(self.mapInfo.width, self.mapInfo.height)
        xmin = max(x_posPxl-radius, 0)
        xmax = min(x_posPxl+radius, self.mapInfo.width - 1)
        ymin = max(y_posPxl-radius, 0)
        ymax = min(y_posPxl+radius, self.mapInfo.height - 1)

        print("xmin, xmax, ymin, ymax: ", xmin, xmax, ymin, ymax)
        print("mapArray2d.shape:", self.mapArray2d.shape)
        for xPxl in np.linspace(xmin, xmax, xmax-xmin):
            for yPxl in np.linspace(ymin, ymax, ymax-ymin):
                xPxl=int(xPxl)
                yPxl=int(yPxl)
                pxl = (xPxl, yPxl)
                if self.mapArray2d[xPxl][yPxl] > 80:
                    self.mark_range_unreachable(pxl, radius)
                elif self.mapArray2d[xPxl][yPxl] == -1: # if pixel is unexplored
                    unexplored_in_range.append(pxl)
        return unexplored_in_range
    
    def waypoint_check_reachable(self, unexplored_list):
        """
        Checks a list of unexplored map pxls for a pxl that is not in the list of known unreachable pixels
        and that a path can be generated to, and returns the coordinates of that pixel.
        """
        print("waypoint_check_reachable - Getting unexplored pixel from unexplored_list.")
        #print("waypoint_check_reachable - unexplored_list: \n", unexplored_list)
        for i in range(len(unexplored_list)):
            waypointPxl = unexplored_list.pop()                    # Choose a pixel at random
            if not self.is_waypointPxl_unreachable(waypointPxl):   # if pixel is not in list of known unreachable pixels
                waypoint = self.coord_pxl2m(waypointPxl)            
                if self.try_generate_path(waypoint):               # and if attempt to generate a path succeeds
                    return waypointPxl                                # Return pixel coords
                
                self.mark_waypointPxl_unreachable(waypointPxl)      # Pixel not marked unreachable, but is -> mark as unreachable
            # Randomly selected pixel is unreachable -> try another
        # Every element of the list is unreachable -> return None
        return None

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