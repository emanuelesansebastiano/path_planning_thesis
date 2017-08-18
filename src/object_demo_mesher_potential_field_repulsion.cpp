 /*Author: Emanuele Sansebastiano */

// my libs
#include <path_planning_thesis/lib_path_planning.h>
#include <moveit_path_planner/lib_path_planner.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "object_demo_mesher_potential_field_repulsion");
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
	std::cout << "Welcome to the demo mesher generating the potential field around the obstacles!" << std::endl;

	//object demo definition (user definition)
	success = false;
	while(!success)
	{
		std::cout << "Which object do you want to see meshed?" << std::endl;
		std::cout << "Enter '1' for the demo SPHERE" << std::endl;
		std::cout << "Enter '2' for the demo BOX" << std::endl;
		std::cout << "Enter '3' for the demo CYLINDER" << std::endl;
		std::cout << "Enter '4' for the demo CONE" << std::endl;
		std::cout << "Enter '5' to quit the program" << std::endl;

		std::cin >> chosen_num;
		if(chosen_num == 1){
			temp_str = "Are you sure you selected the SPHERE?";
			success = thf::yes_not(temp_str);
			if(success)
				obj2pub = mbf::collision_obj_generator("SPHERE_demo", msf::makeVector3(1.443,0.0,0.0), 0.199);
		}else if(chosen_num == 2){
			temp_str = "Are you sure you selected the BOX?";
			success = thf::yes_not(temp_str);
			if(success)
				obj2pub = mbf::collision_obj_generator("BOX_demo", msf::makeVector3(1.443,0.0,0.0), msf::makeVector3(45.0,45.0,45.0), msf::makeVector3(0.35,0.24,0.67));
		}else if(chosen_num == 3){
			temp_str = "Are you sure you selected the CYLINDER?";
			success = thf::yes_not(temp_str);
			if(success)
				obj2pub = mbf::collision_obj_generator("CYLINDER_demo", msf::makeVector3(1.443,0.0,0.0), msf::makeVector3(45.0,45.0,45.0), 0.67,0.15);
		}else if(chosen_num == 4){
			temp_str = "Are you sure you selected the CONE?";
			success = thf::yes_not(temp_str);
			if(success)
				obj2pub = mbf::collision_obj_generator("CONE_demo", msf::makeVector3(1.443,0.0,0.0), msf::makeVector3(45.0,45.0,45.0), 0.47,0.15, "CONE");
		}else if(chosen_num == 5){
			temp_str = "Are you sure you want to quit the program?";
			success = thf::yes_not(temp_str);
			if(success){
				ros::shutdown();
				return 0;}
		}else{
			std::cout << "Error: you entered an invalid value..." << std::endl;
		}
	}

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

	std::vector< std::vector< std::vector< double > > > cellmap =  plf::mapCell_generator(4.0, 4.0, 4.0, cell_size);

	//information on the map
	std::cout << "Since your cell size is " << cell_size << ", the planning workspace is composed by the following number of cells:" << std::endl;
	std::cout << cellmap.size() << " x " << cellmap[0].size() << " x " << cellmap[0][0].size() << " = " << cellmap.size() * cellmap[0].size() * cellmap[0][0].size() << std::endl;
	std::cout << "The world center is located in the cell:" << std::endl;
	std::cout << cellmap.size()/2 << " | " << cellmap[0].size()/2 << " | " << cellmap[0][0].size()/2 << std::endl;

	//publish object selected into the scene
	of::addObject(planning_scene_interface, obj2pub);

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

	//potential field section
	double radius_pf;
	double coord[3];
	double dim_temp[3];
	temp_str = "Do you want to proceed with the potential field visualization?";
	success = thf::yes_not(temp_str);
	if(!success){
		ros::shutdown();
		return 0;}

	success = false;
	while(!success)
	{
		std::cout << "How large do you want to make the potential field radius?" << std::endl;
		std::cout << "Remember: the cell size is: " << cell_size << std::endl;

		std::cin >> radius_pf;

		sprintf(numstr, "%f", radius_pf);
		temp_str = "Are you sure you want to use ";
		temp_str += numstr;
		temp_str += " as potential field radius?";
		success = thf::yes_not(temp_str);
	}

	cellmap = plf::mapCell_potential_field_repulsion(cellmap,cell_size,radius_pf);

	std::vector<std::vector<int>> coor_matrix = plf::mapCell_extractor_IJK(cellmap, 0.00001);

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
