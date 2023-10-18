note# METR4202
Group project for metr4202 course.
This code will autonomously operate a turtlebot robot in using ROS and gazebo, and using SLAM for mapping and navigation.

Team members:
--------------
Jonathan Trevatt - jonathan@trevatt.net - 43538706\
Tom (Chun Yu) NG - chun.y.ng@uqconnect.edu.au - 45685211\
Isaiah Stook - isaiahstook@gmx.com - 44539120\
Pei-Jer Tsai - p.tsai@uqconnect.edu.au - 46411172

## Build and run instructions:
### In powershell
**Start from clean environment:**
```bash
wsl --shutdown
```

### In WSL ubuntu environment:
```bash
# Create a new empty workspace directory in the home folder:
cd ~
mkdir --p metr4202_ws/src
cd metr4202_ws/src

# Download repository into src folder:
git clone https://github.com/Darkspore52/tb3_controller.git

# Build and source files:
source /opt/ros/humble/setup.bash
cd .. # To metr4202_ws
colcon build
source install/setup.bash
```

### In a fresh ubuntu terminal - Launch gazebo map
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
```bash
# Or, for other world files:
ros2 launch worlds/world_name.launch.py
```

### In a fresh ubuntu terminal - Launch rviz/nav2 with SLAM:
```bash
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py slam:=True
```

### In a fresh ubuntu terminal - Run the controller node:
```bash
# Source setup.bash and underlay ros evironment
source /opt/ros/humble/setup.bash
source metr4202_ws/src/install/setup.bash
# Run our node
ros2 run tb3_controller turtlebot_brain
```
------------------
------------------

Instructions to collaborate:
------------------
To gain commandline access to the repository (by setting up a key):
* Github account settings -> developer settings -> personal access tokens -> Tokens (classic) -> generate new token (classic)
* Give name for key (e.g. "metr4202")
* Set expiration to 90 days
* Tick all permissions 'on'
* Click 'Generate'
* This gives you a password to use to access git via commandline

```bash
git config --global credential.helper store #To save credentials (WARNING - in plain text!) 
git config --global user.name "Your Name"
git config --global user.email "your_email@whatever.com"
git config --global core.autocrlf true
git config --global core.safecrlf warn
git clone https://github.com/Darkspore52/tb3_controller.git # Clone repository
```

Instructions to commit changes (for reference):
----------------------
```bash
git add .
git commit -m "Comment"
git push
```

Common ros commands:
--------------
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export GAZEBO_MODEL_PATH=~/.gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/turtlebot3_simulations/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch slam_toolbox online_async_launch.py
ros2 run turtlebot3_teleop teleop_keyboard
rviz2

# Save map:
cd && mkdir maps
ros2 run nav2_map_server map_saver_cli -f ~/turtlebot3_ws/maps/turtlebot3_world_map

#Open map:
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/turtlebot3_ws/maps/turtlebot3_world_map.yaml
```

Resources:
--------------------
git tutorial - https://githowto.com/
Turtlebot3 emanual - https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
Nav2 - https://navigation.ros.org/

Assignment tasks:
------------------
![image](https://github.com/Darkspore52/METR4202/assets/53199626/2ed54762-153d-4e1a-82b4-4402c19c313a)
![image](https://github.com/Darkspore52/METR4202/assets/53199626/f72eb190-d610-42e2-a826-6b94fcd896db)
![image](https://github.com/Darkspore52/METR4202/assets/53199626/133a43dd-382d-42e8-8028-866ece37b6ac)


