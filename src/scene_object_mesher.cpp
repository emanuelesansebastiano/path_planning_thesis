 /*Author: Emanuele Sansebastiano */

// my libs
#include <path_planning_thesis/lib_path_planning.h>
#include <moveit_path_planner/lib_path_planner.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "scene_object_mesher");
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
	int chosen_num;
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
	std::cout << "Welcome to the scene object mesher!" << std::endl;

	//are you are about it? (user definition)
	success = false;
	temp_str = "Are you sure you want to continue?";
	success = thf::yes_not(temp_str);
	if(!success){
		std::cout << "quitting...." << std::endl;
		ros::shutdown();
		return 0;
	}

	//mesh size definition (user definition)
	success = false;
	while(!success)
	{
		std::cout << "Which mesh size do you want to use?" << std::endl;
		std::cout << "(A value include in [0.01; 0.2] is suggested)" << std::endl;

		std::cin >> cell_size;

		if(cell_size <= 0.0)
		{
			std::cout << "Since the mesh size is a physical value, it cannot be negative." << std::endl;
			std::cout << "It has been converted to a positive value" << std::endl;
			cell_size = msf::abs_f(cell_size);
		}

		sprintf(numstr, "%f", cell_size);
		temp_str = "Are you sure you want to use ";
		temp_str += numstr;
		temp_str += " as mesh size?";
		success = thf::yes_not(temp_str);
	}

	double workspace_size = 4.0;
	std::vector< std::vector< std::vector< double > > > cellmap =  plf::mapCell_generator(workspace_size, workspace_size, workspace_size, cell_size);

	//information on the map
	std::cout << "Since your cell size is " << cell_size << ", the planning workspace is composed by the following number of cells:" << std::endl;
	std::cout << cellmap.size() << " x " << cellmap[0].size() << " x " << cellmap[0][0].size() << " = " << cellmap.size() * cellmap[0].size() * cellmap[0][0].size() << std::endl;
	std::cout << "The world center is located in the cell:" << std::endl;
	std::cout << cellmap.size()/2 << " | " << cellmap[0].size()/2 << " | " << cellmap[0][0].size()/2 << std::endl;

	//get objects from the scene
	ros::Duration(0.1).sleep();
	obj_list = of::GetMoveitSceneObjects(planning_scene_interface);

	//map occupancy
	plf::mapCell_objsInsertion(cellmap,cell_size,obj_list);
	plf::mapCell_saturator(cellmap);

	//extract the coordinates of the cells occupied
	blocks_coord = plf::mapCell_extractor(cellmap,cell_size);

	for(int i = 0; i < blocks_coord.size(); i++)
	{
		temp_str = "cell_";
		sprintf(numstr, "%f", blocks_coord[i].x);
		temp_str += numstr;
		sprintf(numstr, "%f", blocks_coord[i].y);
		temp_str += numstr;
		sprintf(numstr, "%f", blocks_coord[i].z);
		temp_str += numstr;

		obj2pub = mbf::collision_obj_generator(temp_str, blocks_coord[i], msf::makeVector3(0.0,0.0,0.0), msf::makeVector3(cell_size,cell_size,cell_size));

		ros::Duration(0.005).sleep();
		of::addObject(planning_scene_interface, obj2pub);
	}


	ros::shutdown();
	return 0;
}
