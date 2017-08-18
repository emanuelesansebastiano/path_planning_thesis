 /*Author: Emanuele Sansebastiano */

// my libs
#include <path_planning_thesis/lib_path_planning.h>
#include <moveit_path_planner/lib_path_planner.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "one_arm_loader");
	ros::NodeHandle node_handle("~");

	ros::AsyncSpinner spinner(1);

	spinner.start();

	namespace msf = moveit_side_functions;
	namespace mbf = moveit_basics_functions;
	namespace gsf = geometry_side_functions;
	namespace of = obj_functions;

    namespace plf = planner_functions;

	namespace thf = thesis_functions;
	namespace brs = baxter_robot_specific;

    //variables
	double cell_size;
	char chosen_char;
	char numstr[30];
	bool success;
	std::string temp_str;

	std::vector<geometry_msgs::Vector3> blocks_coord;
	moveit_msgs::CollisionObject obj2pub;
	std::vector<moveit_msgs::CollisionObject> obj_list;

	//moveit initialization
	static const std::string PLANNING_GROUP = "both_arms";
	moveit::planning_interface::MoveGroup move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	msf::countdown_sec();

	//main program
	std::cout << "Welcome to the one arm loader program!" << std::endl;

	//arm to load (user definition)
	success = false;
	while(!success)
	{
		std::cout << "Which arm to you want to load?" << std::endl;
		std::cout << "Enter 'L' for the left one" << std::endl;
		std::cout << "Enter 'R' for the right one" << std::endl;
		std::cout << "Enter 'Q' to quit the program" << std::endl;

		std::cin >> chosen_char;
		if(chosen_char == 'L' || chosen_char == 'l'){
			temp_str = "Are you sure you selected the left arm?";
			success = thf::yes_not(temp_str);
			if(success)
				obj_list = brs::Baxter_arm_objcts_occupancy("left");
		}else if(chosen_char == 'R' || chosen_char == 'r'){
			temp_str = "Are you sure you selected the right arm?";
			success = thf::yes_not(temp_str);
			if(success)
				obj_list = brs::Baxter_arm_objcts_occupancy("right");
		}else if(chosen_char == 'Q' || chosen_char == 'q'){
			temp_str = "Are you sure you want to quit?";
			success = thf::yes_not(temp_str);
			if(success){
				ros::shutdown();
				return 0;}
		}else{
			std::cout << "Error: you entered an invalid key..." << std::endl;
		}
	}

	of::addObject(planning_scene_interface, obj_list);


	ros::shutdown();
	return 0;
}
