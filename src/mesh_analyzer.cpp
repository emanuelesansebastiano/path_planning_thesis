 /*Author: Emanuele Sansebastiano */

// my libs
#include <path_planning_thesis/lib_path_planning.h>
#include <moveit_path_planner/lib_path_planner.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mesh_analyzer");
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
	double cell_size_max, cell_size_min, cell_size_jump;
	double temp_time1, temp_time2;
	int iterations;
	bool success;
	std::string temp_str;

	std::vector<geometry_msgs::Vector3> blocks_coord;
	std::vector<moveit_msgs::CollisionObject> obj_list;
	std::vector< std::vector< std::vector< double > > > cellmap;


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

	//mesh sizes
	cell_size_max = 0.2;
	cell_size_min = 0.005;
	cell_size_jump = 0.0025;
	iterations = (cell_size_max - cell_size_min)/cell_size_jump +1;

	//get objects from the scene
	ros::Duration(0.1).sleep();
	obj_list = of::GetMoveitSceneObjects(planning_scene_interface);

	//workspace size
	double workspace_size = 4.0;

	std::cout << "iteration | cell size [m] | # cells | # occupied cells | time spent" << std::endl;
	std::cout << std::endl;

	for(int i = 0; i < iterations; i++)
	{
		temp_time1 = msf::getCurrentTime(); //[msec]

		cell_size = cell_size_max - i*cell_size_jump;

		cellmap.clear();
		cellmap =  plf::mapCell_generator(workspace_size, workspace_size, workspace_size, cell_size);

		//map occupancy
		plf::mapCell_objsInsertion(cellmap,cell_size,obj_list);
		plf::mapCell_saturator(cellmap);

		//extract the coordinates of the cells occupied
		blocks_coord = plf::mapCell_extractor(cellmap,cell_size);

		temp_time2 = msf::getCurrentTime() - temp_time1; //[msec]
		temp_time2 /= 1000;

		std::cout << i+1 << "/" << iterations << "	" << cell_size << "	" << cellmap.size()*cellmap[0].size()*cellmap[0][0].size() << "	" << blocks_coord.size() << "	" << temp_time2 << std::endl;
	}


	ros::shutdown();
	return 0;
}
