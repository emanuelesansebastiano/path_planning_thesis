 /*Author: Emanuele Sansebastiano */

// my libs
#include <path_planning_thesis/lib_path_planning.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "scene_loader");
	ros::NodeHandle node_handle("~");

	ros::AsyncSpinner spinner(1);

	spinner.start();

	namespace thf = thesis_functions;

	//scene loader code
	moveit_msgs::CollisionObject obj_temp;
	std::vector<moveit_msgs::CollisionObject> obj_list =  moveit_basics_functions::readCollisionObj(thf::file_handler_r());

	moveit::planning_interface::PlanningSceneInterface interface;

	std::string question = "Do you want to give some specific color to the loaded objects?";
	std::string cin_color;

	if(thf::yes_not(question))
	{
		std::cout << "Enter the color you want..." << std::endl;
		std::cin.ignore();
		std::getline(std::cin,  cin_color);
		obj_functions::addObject(interface,obj_list,cin_color);
	}else{
		obj_functions::addObject(interface,obj_list);
	}

	ros::shutdown();
	return 0;
}
