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
import typing

class Brain(Node):
    def __init__(self):
        """
        Initialization function for the Brain class, which sets up various
        subscriptions and publishers, as well as initializes flags and variables.
        """
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
    def odom_callback(self, msg:Odometry) -> None:
        """
        Called whenever a new odometry message is published to the 'odom' topic.
        Updates global variables with the current position of the robot and 
        sends the first waypoint to move to if the map is ready.
        
        Args:
          msg (Odometry): Message object that contains information about the 
          robot's odometry, such as its position and orientation.
        """
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
        
    def map_callback(self, msg:OccupancyGrid) -> None:
        """
        Called whenever a new OccupancyGrid message is published to the 'map' topic.
        Updates global variables with the new map data, after converting to a 2D array.
        Initializes an array for unreachable positions if necessary. 
        Initializes and publishes the map of known unreachable pixels.
        Sets the ready_map flag to True.
        
        Values range [-1, 100], where -1 represents an unknown probablility.
        
        Args:
          msg (OccupancyGrid): The parameter `msg` is of type `OccupancyGrid`, which represents a 2-D 
          grid map, in which each cell represents the probability of occupancy. Each cell can take 
          values of -1 (unknown), or [0-100] (the % probability that the cell is occupied).

        Returns:
          nothing (None).
        """
        print('NOTE - turtlebot_brain.map_callback: reached')
        self.mapMsg = msg
        self.mapArray2d = np.reshape(msg.data, (-1, msg.info.width))
        self.mapInfo = msg.info
        if self.unreachable_positions == []:
            self.unreachable_positions = np.zeros((msg.info.height, msg.info.width), dtype=bool)
        
        if not self.map_unreachable_initFlag:
            self.init_map_unreachable(msg)
            self.map_reachable_publisher.publish(self.map_unreachable)

        self.ready_map = True
        self.map_reachable_publisher.publish(self.map_unreachable)
        return
    
    def path_callback(self, msg:Path) -> None:
        """
        Called whenever a new Path message is published to the 'local_plan' topic.
        Saves the path as the global variable self.path.
        
        Args:
          msg (Path): The parameter `msg` is of type `Path`.
        """
        self.path = msg
    
    def init_map_unreachable(self, msg:OccupancyGrid) -> None:
        """
        The function initializes a new OccupancyGrid called "map_unreachable" with the same header and info
        as the subscribed map, and sets the data values to either 0 or 100 based on whether the in the subscribed
        map is >80 (% chance that it is occupied).
        
        Args:
          msg (OccupancyGrid): The parameter `msg` is of type `OccupancyGrid`. It is used to initialize the
        `map_unreachable` attribute of the class.
        
        Returns:
          `map_unreachable` object.
        """
        self.map_unreachable_initFlag = True

        self.map_unreachable = OccupancyGrid()
        self.map_unreachable.header.frame_id = "map"
        self.map_unreachable.info = msg.info
        size = np.array(msg.data).size
        map_unreachable_data = array('b', [int(xv) if c else 101*int(yv) for c, xv, yv in zip(np.array(msg.data) > 80, np.zeros(size), np.ones(size))])

        self.map_unreachable.data = map_unreachable_data
        return

    def bt_log_callback(self, msg:BehaviorTreeLog) -> None:
      """
      Called whenever a new BehaviorTreeLog message is published to the 'behavior_tree_log' topic.
        This is the start of the logic loop that computes points and commands the robot.
        Checks the log for the 'IDLE' event. If the robot is not busy, and the odometry and map data 
        is initialised, it will compute a waypoint and command the robot to move to it.
        Otherwise it prints "robot busy".
      
      Args:
        msg (BehaviorTreeLog): The parameter `msg` is an instance of the `BehaviorTreeLog` class. It
      contains information about the behavior tree log, including the event log.
      """
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

    def coord_pxl2m(self, waypointPxl: tuple[int, int]) -> tuple[float, float, float]:
        """
        Converts pixel coordinates (in the map frame) to meter coordinates (in the world frame)
        using the map resolution and origin position.
        
        Args:
          waypointPxl (tuple[int, int]): The coordinates of a waypoint as a pixel coordinate. 
          Represented as a tuple of integer x and y coordinates of the waypoint on the map.
        
        Returns:
          waypoint (tuple[float, float, float]): The coordinates of a waypoint as coordinates in meters in the world frame. 
          Represented as a tuple of 3 floats: the x and y coordinates in meters, and w,
          the bearing (as a quaternion) which is arbitrarily set to 0.
        """
        mapPos_x, mapPos_y = waypointPxl
        pos_x = (mapPos_x * self.mapInfo.resolution) + self.mapInfo.origin.position.x
        pos_y = (mapPos_y * self.mapInfo.resolution) + self.mapInfo.origin.position.y
        pos_w = 0 # Arbitrary quaternion bearing
        waypoint = (pos_x, pos_y, pos_w)
        return waypoint

    def coord_m2pxl(self, waypoint: tuple[float, float, float]) -> tuple[int, int]:
        """
        Converts a waypoint's coordinates from meters (in the world frame) to pixels 
        (in the map frame) based on the map's resolution and origin position.
        
        Args:
          waypoint (tuple[float, float, float]): The coordinates of a waypoint as coordinates in meters in the world frame. 
          Represented as a tuple of 3 floats: the pos_x and pos_y coordinates in meters, and pos_w, 
          the bearing (as a quaternion).
        
        Returns:
          waypointPxl (tuple(in, int)): The coordinates of a waypoint as a pixel coordinate. 
            Represented as a tuple of integer x and y coordinates of the waypoint on the map.
        """
        pos_x, pos_y, pos_w = waypoint
        mapPos_x = int((pos_x - self.mapInfo.origin.position.x)/self.mapInfo.resolution)
        mapPos_y = int((pos_y - self.mapInfo.origin.position.y)/self.mapInfo.resolution)
        waypointPxl = (mapPos_x, mapPos_y)
        return waypointPxl
    
    def get_coords_as_Pxl(self) -> tuple[int, int]:
        """
        Returns the current position as the x,y pixel position on the map.
        Returns:
          coord_m2pxl (tuple(in, int)): the coordinates of the waypoint in pixels.
        """
        waypoint = (self.pos_x, self.pos_y, self.pos_w)
        return self.coord_m2pxl(waypoint)
    
    # TODO - needs testing, may not work
    def mark_range_unreachable(self, pxl: tuple[int, int], radius: int) -> None:
        """
        Marks a given pixel, along with a given range around that pixel, as known unreachable.
        I.e., the robot cannot or should not attempt to pathfind to them.
        
        Args:
          pxl (tuple[int, int]): A pixel position on a map, around which to mark unreachable.
          radius: The radius parameter represents the distance from the center pixel (pxl) within which
                    we want to mark the range as unreachable.
        """
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

    def mark_waypointPxl_unreachable(self, waypointPxl: tuple[int, int]) -> None:
        """
        Marks a given waypoint pixel as unreachable.
        I.e., the robot cannot or should not attempt to pathfind there.
        
        Args:
          waypointPxl (tuple[int, int]): The pixel coordinates of a waypoint to mark as unreachable.
        
        Returns:
          nothing (None).
        """
        print("Marking waypoint as unreachable.")
        xPxl, yPxl = waypointPxl
        self.unreachable_positions[xPxl][yPxl] = True
        return

    def mark_area_unreachable(self, waypointPxl: tuple[int, int]) -> None:
        """
        Marks a specified waypoint as unreachable by setting the corresponding positions in the
        `unreachable_positions` array to `True`.
        Similar to `mark_range_unreachable`.
        
        Args:
          waypointPxl (tuple[int, int]): The x, y pixel coordinates of a waypoint on a map.
        
        Returns:
          nothing (None).
        """
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

    def is_waypointPxl_unreachable(self, waypointPxl: tuple[int, int]) -> bool:
        """
        Checks if a given waypoint pixel has been marked as unreachable based on a 2D array of unreachable positions.

        Args:
          waypointPxl (tuple[int, int]): The coordinates (x, y) of a waypoint in pixel units.
        
        Returns:
          self.unreachable_positions[xPxl][yPxl] (bool): 
          The value at the position (xPxl, yPxl) in the 2D list self.unreachable_positions.
        """
        xPxl, yPxl = waypointPxl
        return self.unreachable_positions[xPxl][yPxl]

    def try_generate_path(self, waypoint: tuple[float, float, float]):
        """
        Tries to generate a path between the current pose and a given waypoint using the
        navigation module. 
        On failure, returns False, otherwise, returns True.
        
        Args:
          waypoint (tuple[float, float, float]): The (x, y, w) waypoint tuple to try to generate a path to.
        
        Returns:
          (bool): If the path is successfully generated, it returns True. Otherwise, it returns False.
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
    
    def waypoint_compute_trivial(self) -> tuple[float, float, float]:
        """
        Returns a waypoint with coordinates (0.5, 0.5, 1).
        Used as a trivial case for testing and marking purposes.
        
        Returns:
          waypoint (tuple[float, float, float]): a n (x,y,w) waypoint tuple (0.5, 0.5, 1).
        """
        waypoint = (0.5, 0.5, 1)
        return waypoint

    def waypointPxl_compute(self) -> tuple[int, int]:
        """
        Searches for unexplored pixels within a radius around the current robot position and returns
        the first reachable unexplored pixel, or None if no valid points are found.

        Returns:
          reachable_waypoint_pxl (tuple(int,int)|None): The (x,y) pixel coordinates of a valid point (if found), otherwise None.
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

    def move_to_waypoint(self, waypoint: tuple[float, float, float]):
        """
        Moves the robot to a specified waypoint (in meters in the global frame) and handles cancellation or failure cases.
        
        Args:
          waypoint (tuple[float, float, float]): The waypoint (x,y,w) tuple to move to.
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
    
    def move_to_waypointPxl(self, waypointPxl: tuple[int, int]):
        """
        Moves the robot to a specified waypoint (in integer map pixel coordinates) and handles cancellation or failure cases.
        
        Args:
          waypoint (tuple(int,int)): The map pixel waypoint (x,y) tuple to move to.
        """
        waypoint = self.coord_pxl2m(waypointPxl)
        self.move_to_waypoint(waypoint)

    def map_get_unexplored_in_range(self, radius: int):
        """
        Returns a list of unexplored pixels within a given radius from the current position.
        
        Args:
          radius (int): The distance from the current position in pixels. It defines the range 
          within which the function will search for unexplored pixels.
        
        Returns:
          unexplored_in_range (list(tuple(int,int))): a list of unexplored pixels (as x,y map coordinate tuples) within a given range.
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
                    self.mark_range_unreachable(pxl, 10)
                elif self.mapArray2d[xPxl][yPxl] == -1: # if pixel is unexplored
                    unexplored_in_range.append(pxl)
        return unexplored_in_range
    
    def waypoint_check_reachable(self, unexplored_list: list[tuple[int, int]]) -> tuple[int, int] | None:
        """
        Checks if a randomly selected pixel from the unexplored_list is reachable and 
        returns the pixel coordinates if it is, otherwise marks the pixel as unreachable
        and tries another random pixel until all pixels in the list have been checked.
        
        Args:
          unexplored_list (list(tuple(int,int))): The `unexplored_list` parameter is a 
          list of pixels that have not been explored yet. 
          Each pixel in the list represents a location that the code needs to check for reachability.
        
        Returns:
          waypointPxl (tuple(int,int)|None): the (x,y) tuple pixel coordinates of a reachable waypoint 
          (if one is found), otherwise returns None.
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

def main(args=None) -> None:
    """
    Entrypoint for the ROS node.
    Initializes the rclpy library, creates an instance of the Brain class, spins the
    brain node, destroys the brain node, and shuts down rclpy.
    
    Args:
      args: Passes command-line arguments to the `main` function. 
      Optional parameter that defaults to `None`. 
      Can be used to configure or customize the behavior of the program.
    """
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
    return

# If the current module is being run as the main program, call the `main()` function.
if __name__ == '__main__':
    main()