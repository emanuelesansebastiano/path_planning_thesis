 /*Author: Emanuele Sansebastiano */

// my libs
#include <path_planning_thesis/lib_path_planning.h>
#include <moveit_path_planner/lib_path_planner.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "full_demo");
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
	double ws_dim[3];
	std::string temp_str;
	std::string arm_rl;

	double attractive_wave_step;
	double radius_repulsive_field;
	geometry_msgs::Vector3 goal_location;


	std::vector<geometry_msgs::Vector3> blocks_coord;
	std::vector<moveit_msgs::CollisionObject> obj_list;
	std::vector<moveit_msgs::CollisionObject> obj_list_temp;
	std::vector<moveit_msgs::CollisionObject> obj_list_baxter_body;
	std::vector<moveit_msgs::CollisionObject> obj_list_arm;
	std::vector< std::vector< std::vector< double > > > cellmap;


	//moveit initialization
	static const std::string PLANNING_GROUP = "right_arm";
	moveit::planning_interface::MoveGroup move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	msf::countdown_sec();

	//main program
	std::cout << "Welcome to the full demo!" << std::endl;

	//are you are about it? (user definition)
	success = false;
	temp_str = "Are you sure you want to continue?";
	success = thf::yes_not(temp_str);
	if(!success){
		std::cout << "quitting...." << std::endl;
		ros::shutdown();
		return 0;
	}

	//loading object in the scene from a file
	std::string sub_folder = "/scenes";

	success = false;
	temp_str = "Do you want to insert some objects in the scene from a file?";
	success = thf::yes_not(temp_str);
	if(success)
	{
		obj_list =  mbf::readCollisionObj(thf::file_handler_r(sub_folder));
		of::addObject(planning_scene_interface,obj_list);

		//inserting another object
		while(success)
			temp_str = "Do you want to insert other objects in the scene from a file?";
			success = thf::yes_not(temp_str);
			if(success)
			{
				obj_list =  mbf::readCollisionObj(thf::file_handler_r(sub_folder));
				of::addObject(planning_scene_interface,obj_list);
			}
	}

	std::cout << "The robot collision has been considered..." << std::endl;

	std::cout << move_group.getName() << std::endl;


	if(PLANNING_GROUP == "right_arm")
		arm_rl = "left";
	else if(PLANNING_GROUP == "left_arm")
		arm_rl = "right";
	else
		std::cout << "Error: This demo has been planned to move just one arm. arms are not involved in the obstacle avoidance." << std::endl;

	//Baxter torso avoidance introduction
	//It come from a specific file contained in the folder scene. It will not be published in the scene (it is just a logic presence) has the arm
	temp_str = sub_folder + "/file_scene_robot_occupancy.txt";
	obj_list_baxter_body = mbf::readCollisionObj(temp_str);

	//map generation
	cell_size = 0.1;
	ws_dim[0] = 4.0;
	ws_dim[1] = 4.0;
	ws_dim[2] = 4.0;

	cellmap = plf::mapCell_generator(ws_dim[0], ws_dim[1], ws_dim[2], cell_size);

	temp_time1 = msf::getCurrentTime();

	//generation of list of the objects to insert
	obj_list.clear();
	obj_list = of::GetMoveitSceneObjects(planning_scene_interface);
	obj_list_arm = brs::Baxter_arm_objcts_occupancy(arm_rl);
	for(int i = 0; i < obj_list_arm.size(); i++)
		obj_list.push_back(obj_list_arm[i]);
	for(int i = 0; i < obj_list_baxter_body.size(); i++)
		obj_list.push_back(obj_list_baxter_body[i]);

	//object insertion in the map and relative saturation
	plf::mapCell_objsInsertion(cellmap, cell_size, obj_list);
	plf::mapCell_saturator(cellmap, 1.0);

	temp_time2 = msf::getCurrentTime() - temp_time1;
	temp_time2 /= 1000;

	std::cout << "The occupancy map has been generated in: " << temp_time2 << " seconds" << std::endl;
	std::cout << "The mesh size is: " << cell_size << ", and it is composed by:	" << cellmap.size()*cellmap[0].size()*cellmap[0][0].size() << " cells" << std::endl;

	// potential field map generator
	attractive_wave_step = 0.1;
	radius_repulsive_field = 0.12;
	goal_location.x = 0.7;
	goal_location.y = 0.0;
	goal_location.z = 0.2;
	std::vector< std::vector< std::vector< double > > > ptf_map = plf::mapCell_potential_field_full(cellmap, cell_size, attractive_wave_step, radius_repulsive_field, goal_location, 1.0);



	//To be continued!


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
