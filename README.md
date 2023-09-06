# METR4202
Group project for metr4202 course
This code will autonomously operate a turtlebot robot in using ROS and gazebo, and using SLAM for mapping and navigation.

Team members:
--------------
Jonathan Trevatt - jonathan@trevatt.net - 43538706\
Tom (Chun Yu) NG - chun.y.ng@uqconnect.edu.au - 45685211\
Isaiah Stook - isaiahstook@gmx.com - 44539120\
Pei-Jer Tsai - p.tsai@uqconnect.edu.au - 46411172

Instructions to download code to ROS workspace:
------------------
First time:
Github account settings -> developer settings -> personal access tokens -> Tokens (classic) -> generate new token (classic)\
Give name (e.g. "metr4202")\
Set expiration to 90 days\
Tick all permissions on\
Generate\
This gives you a password to use to access git via commandline

To save credentials (WARNING - in plain text!): git config --global credential.helper store

git config --global user.name "Your Name"

git config --global user.email "your_email@whatever.com"

git config --global core.autocrlf true

git config --global core.safecrlf warn

git clone https://github.com/Darkspore52/METR4202.git

Instructions to commit changes (for reference):
----------------------
git add .

git commit -m "Comment"

git push

Common ros commands:
--------------
source /opt/ros/humble/setup.bash

source install/setup.bash

export GAZEBO_MODEL_PATH=~/.gazebo/models:$GAZEBO_MODEL_PATH

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/turtlebot3_simulations/turtlebot3_gazebo/models

export TURTLEBOT3_MODEL=waffle_pi

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

ros2 launch slam_toolbox online_async_launch.py

ros2 run turtlebot3_teleop teleop_keyboard

rviz2

Save map:

cd && mkdir maps

ros2 run nav2_map_server map_saver_cli -f ~/turtlebot3_ws/maps/turtlebot3_world_map

Open map:

ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=$HOME/turtlebot3_ws/maps/turtlebot3_world_map.yaml

Resources:
--------------------
git tutorial - https://githowto.com/
Turtlebot3 emanual - https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
Nav2 - https://navigation.ros.org/

First task:
------------------
![image](https://github.com/Darkspore52/METR4202/assets/53199626/2ed54762-153d-4e1a-82b4-4402c19c313a)
