/* Author: Emanuele Sansebastiano
   Desc: Library to encapsulate some functions useful to accomplish the "path Planning" final project based on Baxter.
         Amazon Robotics Challenge - Universidad Jaume I (Spain) Competitor
*/

#ifndef PATH_PLANNING_THESIS_H
#define PATH_PLANNING_THESIS_H

// ROS
#include <ros/ros.h>

// C++
#include <geometry_msgs/PoseStamped.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <stdio.h>
#include <std_msgs/Bool.h>

// Unofficial Moveit! libraries
#include <moveit_side_pkg/side_functions.h>


//////////////////////////////////////////////////////////////////////////////////////
// VALUES MODIFIABLE BY THE USER \\

//brief: change this string to change the directory of the file from you get the information
const std::string script_directory_string = "/home/emanuele/emanuele_stuff/Programming_courses/baxter_ws/src/path_planning_thesis/data_file";
const std::string file_type = ".txt";

//////////////////////////////////////////////////////////////////////////////////////


namespace thesis_functions
{
  //brief: function to express "yes" or "not"
  bool yes_not(std::string question);

  //brief: function to check if a file exist using the whole directory
  bool existFile(std::string directoryName);

  //brief: function to handle the file name of which the users want to write;
  //	   sub_directory must be similar to "/folder1/folder2/..."
  std::string file_handler_r(std::string sub_directory = "/");

  //brief: function to handle the file name of which the users want to write
  //	   sub_directory must be similar to "/folder1/folder2/..."
  std::string file_handler_w(std::string sub_directory = "/");

  //brief: function to read all the poses contained in a file
  std::vector<geometry_msgs::Pose> read_pose(std::string directory_file);

  //brief: function to save a sequence of end_effector positions
  void write_seq_pose(ros::NodeHandle &nh);

  //brief: function to get the initial joint position of my experiments
  //       it is a 14 size vector (first 7 left; second 7 right)
  std::vector<double> get_ititial_joint_value();
  //brief: function to get a shaked initial joint position
  std::vector<double> get_shake_ititial_joint_value_up();
  //brief: function to get a shaked initial joint position
  std::vector<double> get_shake_ititial_joint_value_left_right();

  //brief: function to activate or deactivate the vacuum gripper
  void vacuum_on_off(ros::NodeHandle &nh, bool active);

// End namespace "thesis_functions"
}


#endif /* PATH_PLANNING_THESIS_H */


