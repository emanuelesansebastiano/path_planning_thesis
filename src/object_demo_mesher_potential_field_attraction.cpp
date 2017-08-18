 /*Author: Emanuele Sansebastiano */

// my libs
#include <path_planning_thesis/lib_path_planning.h>
#include <moveit_path_planner/lib_path_planner.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_demo_mesher_potential_field_attraction");
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
	char numstr[30];
	bool success;
	std::string temp_str;
	geometry_msgs::Vector3 goal_location;

	moveit_msgs::CollisionObject obj2pub;
	std::vector<moveit_msgs::CollisionObject> obj_list;


	//moveit initialization
	static const std::string PLANNING_GROUP = "both_arms";
	moveit::planning_interface::MoveGroup move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	msf::countdown_sec();

	//main program
	std::cout << "Welcome to the demo mesher generating the potential field sinking into the goal location!" << std::endl;

	//user initializer
	goal_location = msf::makeVector3(1.0, 1.0, 0.0);
	std::cout << "This program will erase all the objects in the scene and show you the potential field referred to the goal position:" ;
	std::cout << goal_location.x << " | " << goal_location.y << " | " << goal_location.z << std::endl;

	temp_str = "Do you want to continue?";
	success = thf::yes_not(temp_str);
	if(!success){
		ros::shutdown();
		return 0;
	}

	std::cout << "The goal location is represented by a cube as big as the mesh size, while the potential field by gradient spheres" << std::endl;


	//all scene object removed
	std::cout << "The program is removing all the object in the scene..." << std::endl;
	obj_list = of::GetMoveitSceneObjects(planning_scene_interface);
	of::removeObject(planning_scene_interface, obj_list);

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

	//map generation
	std::vector< std::vector< std::vector< double > > > cellmap =  plf::mapCell_generator(4.0, 4.0, 4.0, cell_size);

	//information on the map
	std::cout << "Since your cell size is " << cell_size << ", the planning workspace is composed by the following number of cells:" << std::endl;
	std::cout << cellmap.size() << " x " << cellmap[0].size() << " x " << cellmap[0][0].size() << " = " << cellmap.size() * cellmap[0].size() * cellmap[0][0].size() << std::endl;
	std::cout << "The world center is located in the cell:" << std::endl;
	std::cout << cellmap.size()/2 << " | " << cellmap[0].size()/2 << " | " << cellmap[0][0].size()/2 << std::endl;

	//publish object selected into the scene
	obj2pub = mbf::collision_obj_generator("BOX_goal", goal_location, msf::makeVector3(0.0, 0.0, 0.0), msf::makeVector3(cell_size, cell_size, cell_size));
	of::addObject(planning_scene_interface, obj2pub);

	ros::Duration(0.1).sleep();

	//new map based on the potential field
	double wave_step = 0.1;
	cellmap = plf::mapCell_potential_field_attraction(cellmap, cell_size, wave_step, goal_location);
	plf::mapCell_normalization(cellmap,1.0);

	//potential field section
	temp_str = "Do you want to proceed with the potential field visualization?";
	success = thf::yes_not(temp_str);
	if(!success){
		ros::shutdown();
		return 0;
	}

	double coord[3];
	double dim_temp[3];
	std::vector<int> temp_vec_dim; temp_vec_dim.resize(3);
	std::vector<std::vector<int>> coor_matrix;

	//points defining the plane [x][y][0];
	for(int h = 0; h < cellmap.size(); h++)
	{
		for(int q = 0; q < cellmap[0].size(); q++)
		{
			temp_vec_dim[0] = h;
			temp_vec_dim[1] = q;
			temp_vec_dim[2] = goal_location.z/cell_size + cellmap[0][0].size()/2;
			coor_matrix.push_back(temp_vec_dim);
		}
	}


	//points defining the plane [x][0][z];
	for(int h = 0; h < cellmap.size(); h++)
	{
		for(int q = 0; q < cellmap[0][0].size(); q++)
		{
			temp_vec_dim[0] = h;
			temp_vec_dim[1] = goal_location.y/cell_size + cellmap[0].size()/2;
			temp_vec_dim[2] = q;
			coor_matrix.push_back(temp_vec_dim);
		}
	}

	dim_temp[0] = cellmap.size()/2;
	dim_temp[1] = cellmap[0].size()/2;
	dim_temp[2] = cellmap[0][0].size()/2;

	for(int i = 0; i < coor_matrix.size(); i++)
	{
		for(int yup = 0; yup < 3; yup++)
			coord[yup] = (coor_matrix[i][yup] - dim_temp[yup]) * cell_size;

		temp_str = "pfs_";
		sprintf(numstr, "%f", coord[0]);
		temp_str += numstr;
		sprintf(numstr, "%f", coord[1]);
		temp_str += numstr;
		sprintf(numstr, "%f", coord[2]);
		temp_str += numstr;

		obj2pub = mbf::collision_obj_generator(temp_str, msf::makeVector3(coord[0], coord[1], coord[2]), cell_size*cellmap[coor_matrix[i][0]] [coor_matrix[i][1]] [coor_matrix[i][2]]/2);

		ros::Duration(0.005).sleep();
		of::addObject(planning_scene_interface, obj2pub);

	}


	ros::shutdown();
	return 0;
}
