import rclpy
import math
from itertools import chain
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.msg import BehaviorTreeLog, Costmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import numpy as np
import random
from array import array
import typing
import copy
class Brain(Node):
    def __init__(self) -> None:
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
        self.init_myMap_flag = False
        self.computing = True
        self.old_map_size = (0,0)
        self.unreachable_positions = np.zeros((1,1), dtype=bool)
        self.last_waypoint_time = self.get_clock().now()
        self.last_move_time = self.get_clock().now()
        self.last_pos = (0, 0, 0)
        self.printOnce_lastString = 'iujoyhk8lkerthd'
        self.printOnce_count = 0
        self.user_started_flag = False

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.SYSTEM_DEFAULT,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)
        
        self.printOnce('NOTE - turtlebot_brain.Brain: instantiating subscriptions')
        self.map_subscription       = self.create_subscription  (OccupancyGrid,             'map',                      self.map_callback,      10)
        self.status_subscription    = self.create_subscription  (BehaviorTreeLog,           'behavior_tree_log',        self.bt_log_callback,   10)
        self.position_subscription  = self.create_subscription  (Odometry,                  'odom',                     self.odom_callback,     10)
        self.path_subscription      = self.create_subscription  (Path,                      'local_plan',               self.path_callback,     10)
        self.waypoint_publisher     = self.create_publisher     (PoseStamped,               'goal_pose',    10)
        self.map_reachable_publisher= self.create_publisher     (OccupancyGrid,             'valid_waypoint_map', qos_profile)

        self.nav = BasicNavigator() # Initialise navigator
        #self.nav.lifecycleStartup() #init_pose = self.cur_pos
        self.printOnce("----------------------------------------------------------------")

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

        if not self.computing and (self.last_move_time + rclpy.time.Duration(seconds = 3)) - self.get_clock().now() < rclpy.time.Duration(seconds = 0):
            self.last_move_time = self.get_clock().now()
            if math.sqrt((self.last_pos[0] - msg.pose.pose.position.x) ** 2 + (self.last_pos[1] - msg.pose.pose.position.y) ** 2) > 0.05 or \
                        abs(msg.pose.pose.orientation.w - self.last_pos[2]) > 0.05:
                pass
            else:
                self.mark_range_unreachable(self.coord_m2pxl((self.pose.pose.position.x, self.pose.pose.position.y, self.pose.pose.orientation.w)), 3)
                self.nav.cancelTask()
                self.nav_canceled = True
            self.last_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.orientation.w)
        
    def map_callback(self, msg:OccupancyGrid) -> None:
        """
        Called whenever a new OccupancyGrid message is published to the 'map' topic.
        Updates global variables with the new map data, after converting to a 2D array.
        Initializes an array for unreachable positions if necessary. 
        Initializes and publishes the map of known unreachable pixels.
        Sets the ready_map flag to True.
        
        Values range [-1, 100], where -1 represents an unknown probability.
        
        Args:
          msg (OccupancyGrid): The parameter `msg` is of type `OccupancyGrid`, which represents a 2-D 
          grid map, in which each cell represents the probability of occupancy. Each cell can take 
          values of -1 (unknown), or [0-100] (the % probability that the cell is occupied).

        Returns:
          nothing (None).
        """
        self.printOnce('NOTE - turtlebot_brain.map_callback: reached')
        self.mapMsg = msg
        self.mapArray2d = np.reshape(msg.data, (msg.info.width, msg.info.height), order='F')
        self.mapInfo = msg.info
        if self.old_map_size[0] <= msg.info.width and \
                  self.old_map_size[1] <= msg.info.height:
            self.new_unreachable_positions = np.zeros((msg.info.width + 1, msg.info.height + 1), dtype=bool)
            self.printOnce("map width/height: ", msg.info.width, "/", msg.info.height)
            self.printOnce("unreachable_positions.shape x/y: ", self.unreachable_positions.shape[0], "/", self.unreachable_positions.shape[1])
            self.unreachable_positions = np.pad(self.unreachable_positions,
                                            ((0, (msg.info.width - self.old_map_size[0])),
                                             (0, (msg.info.height - self.old_map_size[1]))), mode="constant")
            self.old_map_size = (msg.info.width, msg.info.height)
            np.copyto(self.new_unreachable_positions, self.unreachable_positions)
            self.unreachable_positions = self.new_unreachable_positions
        self.ready_map = True
        
        
        # Initialise custom map
        if not self.init_myMap_flag:
          self.valid_waypoint_map = OccupancyGrid()
          self.init_myMap_flag = True
        # Update map header
        self.valid_waypoint_map.header.frame_id = msg.header.frame_id # id for transfer function for map frame is 'map'
        self.valid_waypoint_map.header.stamp = msg.header.stamp # Or self.get_clock().now().to_msg() ?
        # Update map info (e.g. dimensions)
        self.valid_waypoint_map.info.origin = msg.info.origin
        self.valid_waypoint_map.info.height = msg.info.height
        self.valid_waypoint_map.info.width = msg.info.width
        self.valid_waypoint_map.info.resolution = msg.info.resolution
        self.valid_waypoint_map.info.map_load_time = msg.info.map_load_time
        # Generate custom map data based on current map data
        self.valid_waypoint_map.data = array('b')
        
        # mapArray2d = np.reshape(msg.data, (msg.info.width, msg.info.height), order='F')
        #new_map_array2d = 100*np.multiply(np.where(self.mapArray2d == -1, True, False), np.where(self.mapArray2d >= 80, True, False))
        new_map_array2d = 100*(np.where(self.mapArray2d == -1, True, False))
        valid_waypoint_map = 100*np.ones(np.shape(self.mapArray2d))
        for x in range(np.shape(self.mapArray2d)[0]):
          for y in range(np.shape(self.mapArray2d)[0]):
            surrounding_values = self.get_surrounding_pixel_values(self.mapArray2d, (x,y))
            unknown_pxl_count = 0
            free_pxl_count = 0
            wall_pxl_count = 0
            if surrounding_values != None:
              for val in surrounding_values:
                if val == -1: unknown_pxl_count += 1
                elif ((val >=0) and (val < 80)): free_pxl_count += 1
                else: wall_pxl_count += 1
              # Conditions of surrounding pixels for a valid waypoint (depends on get_surrounding_pixel_values radius)
              if ((unknown_pxl_count>=15) and (free_pxl_count>=10) and (wall_pxl_count == 0)):
                valid_waypoint_map[x][y] = 0
              # mark unreachable positions on map
              if self.unreachable_positions[x][y] == True:
                valid_waypoint_map[x][y] = 80


        new_map_array1d = np.reshape(valid_waypoint_map, (msg.info.width * msg.info.height), order='F')
        
        self.valid_waypoint_map.data = array('b')
        for ele in new_map_array1d:
          ele = int(ele)
          self.valid_waypoint_map.data.append(ele)
        self.map_reachable_publisher.publish(self.valid_waypoint_map)
        
        return

    def path_callback(self, msg:Path) -> None:
        """
        Called whenever a new Path message is published to the 'local_plan' topic.
        Saves the path as the global variable self.path.
        
        Args:
          msg (Path): The parameter `msg` is of type `Path`.
        """
        self.path = msg

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
                  if not self.user_started_flag:
                    self.user_started_flag = True
                    input("Press Enter to continue...")
                  self.computing = True
                  waypointPxl = self.waypointPxl_compute()
                  self.printOnce("waypointPxl: ", waypointPxl)
                  waypoint = self.coord_pxl2m(waypointPxl)
                  self.printOnce("waypoint: ", waypoint)
                  self.computing = False
                  self.last_move_time = self.get_clock().now()
                  self.last_pos = (self.pos_x, self.pos_y, self.pos_w)
                  self.move_to_waypoint(waypoint)
          else: self.printOnce("robot busy")
      self.ready_log = True

    def printOnce(self, *args) -> None:
      """Makes sure that that a print message isn't repeated too many times.
      If a print message is the same as last time this function was called, it doesn't print.

      Args:
          string (str): string to print
      """
      string = ""
      # Construct string to print
      for arg in args:
         string += str(arg)
      
      if string == self.printOnce_lastString:
        self.printOnce_count += 1
      else:
        if self.printOnce_count >= 2:
          print("Printed ", self.printOnce_count, " times.")
        self.printOnce_count = 0
        self.printOnce_lastString = string
        print(string)
      return
    
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
        if waypointPxl != None:
          mapPos_x, mapPos_y = waypointPxl
          pos_x = (mapPos_x) * self.mapInfo.resolution + self.mapInfo.origin.position.x
          pos_y = (mapPos_y) * self.mapInfo.resolution + self.mapInfo.origin.position.y
          pos_w = 1.0 # Arbitrary quaternion bearing
          waypoint = (pos_x, pos_y, pos_w)
          return waypoint
        else:
          self.printOnce("map is done")
          exit()

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
        pos_x, pos_y, _ = waypoint
        mapPos_x = int((pos_x - self.mapInfo.origin.position.x) / self.mapInfo.resolution)
        mapPos_y = int((pos_y - self.mapInfo.origin.position.y) / self.mapInfo.resolution)
        waypointPxl = (mapPos_x, mapPos_y)
        return waypointPxl
    
    # TODO - needs testing, may not work
    def mark_range_unreachable(self, pxl: tuple[int, int], radius: int) -> None:
        """
        Marks a given pixel, along with a given range around that pixel, as known unreachable.
        I.e., the robot cannot or should not attempt to path-find to them.
        
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

        #self.printOnce("mapArray2d.shape:", self.mapArray2d.shape)
        for xPxl in range(xmin, xmax):
            for yPxl in range(ymin, ymax):
                pxl = (int(xPxl), int(yPxl))
                self.mark_waypointPxl_unreachable(pxl)

    def mark_waypointPxl_unreachable(self, waypointPxl: tuple[int, int]) -> None:
        """
        Marks a given waypoint pixel as unreachable.
        I.e., the robot cannot or should not attempt to path-find there.
        
        Args:
          waypointPxl (tuple[int, int]): The pixel coordinates of a waypoint to mark as unreachable.
        
        Returns:
          nothing (None).
        """
        self.printOnce("Marking waypoint as unreachable.")
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
        self.printOnce("Marking waypoint as unreachable.")
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
    
    def get_coords_as_Pxl(self) -> tuple[int, int]:
        """
        Returns the current position as the x,y pixel position on the map.
        Returns:
          coord_m2pxl (tuple(in, int)): the coordinates of the waypoint in pixels.
        """
        waypoint = (self.pos_x, self.pos_y, self.pos_w)
        return self.coord_m2pxl(waypoint)
    
    # takes an OccupancyGrid object, and a pixel coordinate, and returns a list of the values
    # of the surrounding pixels
    def get_surrounding_pixel_values(self, ocgrid: OccupancyGrid | np.dtype, pixel: tuple[int, int], radius: int = 5) -> list[int] | None:
      if type(ocgrid) is OccupancyGrid:
        data_array_2d = np.reshape(ocgrid.data, (ocgrid.info.width, ocgrid.info.height), order='F')
        
      else:
        data_array_2d = ocgrid
        
      x, y = pixel
      pixel_vals = []
      
      radius = 5
      if ((x<=0+radius) or (y<=0+radius) or (x>=np.shape(data_array_2d)[0]-1-radius) or (y>=np.shape(data_array_2d)[1]-1-radius)):
        self.printOnce("Requested pixel is on border of map or outside map.")
        pixel_vals = None
      else:
        for i in range(-radius, radius+1):
          for j in range(-1, 2):
            pixel_vals.append(data_array_2d[x+i][y+j])
      self.printOnce(pixel_vals)
      return pixel_vals
    
    def waypointPxl_compute(self) -> tuple[int, int]:
        """
        Searches for unexplored pixels within a radius around the current robot position and returns
        the first reachable unexplored pixel, or None if no valid points are found.

        Returns:
          reachable_waypoint_pxl (tuple(int,int)|None): The (x,y) pixel coordinates of a valid point (if found), otherwise None.
        """
        
        self.printOnce('NOTE - turtlebot_brain.waypoint_compute: reached')
        min_search_radius = 5
        increment = 1
        max_search_radius = max(self.mapInfo.width, self.mapInfo.height)
        # search radius for reachable, unexplored pixels and set goal to go there
        for search_radius in range(min_search_radius, max_search_radius, increment):
            self.printOnce("waypoint_compute - Searching for unexplored pixels in radius: ", search_radius)
            # generate list of unexplored pixels within search radius
            unexplored_list = self.map_get_unexplored_in_range(search_radius - increment, search_radius)
            self.printOnce("waypoint_compute - unexplored_list: ", unexplored_list)
            unexplored_x_y_coords = []
            for pxl in unexplored_list:
                unexplored_x_y_coords.append(self.coord_pxl2m(pxl))
            if len(unexplored_list) != 0:
                self.printOnce("waypoint_compute - unexplored_list is not Empty - unexplored_list = \n", unexplored_x_y_coords)
                self.printOnce("randomly shuffling unexplored_list to remove preference for exploring in a certain direction.")
                random.shuffle(unexplored_list)
                #self.printOnce("Shuffled list = \n", unexplored_list)
                self.printOnce("waypoint_compute - calling self.waypoint_check_reachable(unexplored_list)")
                reachable_waypoint_pxl = self.waypoint_check_reachable(unexplored_list)
                if reachable_waypoint_pxl is not None:
                    return reachable_waypoint_pxl # Stop searching
        self.printOnce("waypoint_compute - unexplored_list is None, or every unexplored element is unreachable.")
        self.printOnce("Expanding search radius.")
        self.printOnce("waypoint_compute - Maximum search radius is reached.")
        self.printOnce("waypoint_compute - Stopping.")
        return None # If no valid points are found, return None

    def move_to_waypoint(self, waypoint: tuple[float, float, float]):
        """
        Moves the robot to a specified waypoint (in meters in the global frame) and handles cancellation or failure cases.
        
        Args:
          waypoint (tuple[float, float, float]): The waypoint (x,y,w) tuple to move to.
        """
        x, y, w = waypoint
        self.printOnce("Moving to waypoint: ", waypoint)
        self.IDLE = False
        self.pose = PoseStamped()
        self.pose.header.frame_id = 'map'
        self.pose.pose.position.x = waypoint[0]
        self.pose.pose.position.y = waypoint[1]
        self.pose.pose.orientation.w = float(waypoint[2])
        self.waypoint_publisher.publish(self.pose)
        self.last_waypoint_time = self.get_clock().now()
        """
        while not self.nav.isTaskComplete():
          feedback = self.nav.getFeedback()
          if Duration.from_msg(feedback.navigation_time) > Duration(seconds=30.0):
            self.nav_canceled = True
            self.nav.cancelTask()
        result = self.nav.getResult()
        if result == result.CANCELED or result == result.FAILED:
          self.mark_range_unreachable(self.coord_m2pxl(waypoint), 10)
        """
    
    def move_to_waypointPxl(self, waypointPxl: tuple[int, int]):
        """
        Moves the robot to a specified waypoint (in integer map pixel coordinates) and handles cancellation or failure cases.
        
        Args:
          waypoint (tuple(int,int)): The map pixel waypoint (x,y) tuple to move to.
        """
        waypoint = self.coord_pxl2m(waypointPxl)
        self.move_to_waypoint(waypoint)

    def map_get_unexplored_in_range(self, min_radius, max_radius: int):
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
        self.printOnce("Getting unexplored pixels in range.")
        unexplored_in_range = []
        self.printOnce("map width/height: ", self.mapInfo.width, "/", self.mapInfo.height)
        xmin_L = max(x_posPxl - min_radius + 1, 0)
        xmax_L = max(x_posPxl - max_radius, 0)
        xmin_R = min(x_posPxl + min_radius, self.mapInfo.width)
        xmax_R = min(x_posPxl + max_radius + 1, self.mapInfo.width)
        ymin_U = min(y_posPxl + min_radius, self.mapInfo.height)
        ymax_U = min(y_posPxl + max_radius + 1, self.mapInfo.height)
        ymin_D = max(y_posPxl - min_radius + 1, 0)
        ymax_D = max(y_posPxl - max_radius, 0)
        self.printOnce("xmin, xmax, ymin, ymax: ", xmin_L, ", ", xmax_R, ", ", ymin_D, ", ", ymax_U)
        #self.printOnce("mapArray2d.shape:", self.mapArray2d.shape)
        for xPxl in chain(range(xmax_L, xmin_L), range(xmin_R, xmax_R)):
            for yPxl in chain(range(ymax_D, ymin_D), range(ymin_U, ymax_U)):
                pxl = (xPxl, yPxl)
                if self.mapArray2d[xPxl][yPxl] == 100:
                    self.mark_range_unreachable(pxl, 3)
                elif self.mapArray2d[xPxl][yPxl] == -1:
                    nearby_explored_pixels = 0
                    explore = True
                    for x in range(-1, 2):
                        for y in range(-1, 2):
                            if xPxl + x < 0 or yPxl + y < 0:
                                explore = False
                                break
                            elif xPxl + x > self.mapInfo.width - 1 or yPxl + y > self.mapInfo.height - 1:
                                explore = False
                                break
                            elif self.mapArray2d[xPxl + x, yPxl + y] != -1:
                                explore = False
                                break
                        if not explore:
                            break
                    if explore:
                        for x in range(-3, 4):
                            for y in range(-3, 4):
                                if xPxl + x < 0 or yPxl + y < 0:
                                    explore = False
                                    break
                                elif xPxl + x > self.mapInfo.width - 1 or yPxl + y > self.mapInfo.height - 1:
                                    explore = False
                                    break
                                elif self.mapArray2d[xPxl + x, yPxl + y] == 0:
                                    nearby_explored_pixels += 1
                                elif self.is_waypointPxl_unreachable((xPxl + x, yPxl + y)):
                                    explore = False
                                    break
                            if not explore:
                                break
                        if nearby_explored_pixels >= 1:
                            if math.sqrt((pxl[0] - x_posPxl) ** 2 + (pxl[1] - y_posPxl) ** 2) > 10:
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
        self.printOnce("waypoint_check_reachable - Getting unexplored pixel from unexplored_list.")
        #self.printOnce("waypoint_check_reachable - unexplored_list: \n", unexplored_list)
        for _ in range(len(unexplored_list)):
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