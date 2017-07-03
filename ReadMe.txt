# path_planning_thesis
Packaged generated to test various path planning algorithms for the path planning. 

Author: Emanuele Sansebastiano

Github page: https://github.com/emanuelesansebastiano

Date: July, 2017


PROJECT PURPOSE:
.....

PROCESS DESCRIPTION:
.......


HOW TO USE...
Required packages:
1) moveit
2) moveit_msgs
3) moveit_resources
4) srdfdom

!!!WARNING!!!
Do not download every package from git hub singularly. They are continuously updated and the versions could mismatch.
Follow the instructions contained in the official website to install the sources: http://moveit.ros.org/install/source/

|||| INSTRUCTIONS 2016-17 ||||

<Workspace already generated (your_ws_name)>

cd ~/your_ws_name/src
wstool init .
wstool merge https://raw.githubusercontent.com/ros-planning/moveit/indigo-devel/moveit.rosinstall
wstool update
rosdep install --from-paths . --ignore-src --rosdistro indigo
cd ..
catkin_make

||||||||||||||||||||||||||||||

Other packages are required:
1) moveit_robots
2) moveit_side_pkg


Baxter packages:
1) baxter
2) baxter_common
3) baxter_examples
4) baxter_interface
5) baxter_tool
6) robotmodulelib

EXTRA:

To use the simulator you need the following packages:
1) baxter_simulator


ENJOY!!!!!!
