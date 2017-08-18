 /*Author: Emanuele Sansebastiano */

// my libs
#include <path_planning_thesis/lib_path_planning.h>
#include <moveit_path_planner/lib_path_planner.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mesh_size_analyzer_potentialfield");
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
	double time_spent[4];
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
	std::cout << "Welcome to the scene object mesh and potential field analyzer!" << std::endl;

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
	cell_size_min = 0.02;
	cell_size_jump = 0.0025;
	iterations = (cell_size_max - cell_size_min)/cell_size_jump +1;

	//get objects from the scene
	ros::Duration(0.1).sleep();
	obj_list = of::GetMoveitSceneObjects(planning_scene_interface);

	//workspace size
	double workspace_size_x = 4.0;
	double workspace_size_y = 4.0;
	double workspace_size_z = 4.0;

	std::cout << "iteration | cell size [m] | # cells | # occupied cells | time spent mesh | time spent repulsion map | time spent attractive map | total time spent" << std::endl;
	std::cout << std::endl;

	for(int i = 0; i < iterations; i++)
	{
		//map meshing
		temp_time1 = msf::getCurrentTime(); //[msec]

		cell_size = cell_size_max - i*cell_size_jump;

		cellmap.clear();
		cellmap =  plf::mapCell_generator(workspace_size_x, workspace_size_y, workspace_size_z, cell_size);

		//map occupancy
		plf::mapCell_objsInsertion(cellmap,cell_size,obj_list);
		plf::mapCell_saturator(cellmap);

		//extract the coordinates of the cells occupied
		blocks_coord = plf::mapCell_extractor(cellmap,cell_size);

		temp_time2 = msf::getCurrentTime() - temp_time1; //[msec]
		time_spent[0] = temp_time2 /1000;

		//potential field repulsion
		temp_time1 = msf::getCurrentTime(); //[msec]

		std::vector< std::vector< std::vector< double > > > map_rep = plf::mapCell_potential_field_repulsion(cellmap, cell_size, 0.15);

		temp_time2 = msf::getCurrentTime() - temp_time1; //[msec]
		time_spent[1] = temp_time2 /1000;

		//potential field repulsion
		temp_time1 = msf::getCurrentTime(); //[msec]

		std::vector< std::vector< std::vector< double > > > map_att = plf::mapCell_potential_field_attraction(cellmap, cell_size, 0.1, msf::makeVector3(1.0,1.0,0.0));

		temp_time2 = msf::getCurrentTime() - temp_time1; //[msec]
		time_spent[2] = temp_time2 /1000;

		time_spent[3] = time_spent[0] + time_spent[1] + time_spent[2];

		std::cout << i+1 << "/" << iterations << "	" << cell_size << "	" << cellmap.size()*cellmap[0].size()*cellmap[0][0].size() << "	" << blocks_coord.size() << "	" << time_spent[0] << "	" << time_spent[1] << "	" << time_spent[2] << "	" << time_spent[3] << std::endl;
	}


	ros::shutdown();
	return 0;
}
