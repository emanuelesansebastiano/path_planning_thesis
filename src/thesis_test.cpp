 /*Author: Emanuele Sansebastiano */

// my libs
#include <path_planning_thesis/lib_path_planning.h>
#include <moveit_path_planner/lib_path_planner.h>



int main(int argc, char **argv)
{
	ros::init(argc, argv, "thesis_test");
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


	//moveit initialization
	static const std::string PLANNING_GROUP = "both_arms";
	moveit::planning_interface::MoveGroup move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	ros::Duration(2.0).sleep();

	int c_val = 2;
	for(int i = 0; i < c_val; i++)
	{
		std::cout << "countdown: " << c_val-1-i << std::endl;
		ros::Duration(1.0).sleep();
	}


	/*
	geometry_msgs::Vector3 counter_conv;
	geometry_msgs::Vector3 angles;
	angles.x = 109.0;
	angles.y = -38.0;
	angles.z = 5.0;

	geometry_msgs::Vector3 trans;
	trans.z = 2.2;

	std::vector <std::vector<double> > mPi;
	std::vector <std::vector<double> > mPf;
	std::vector <std::vector<double> > mT;
	std::vector <std::vector<double> > mR;
	std::vector <std::vector<double> > mtemp;
	std::vector <std::vector<double> > mtemp2;

	mPi = gsf::point2matrix_gen(trans);

	std::cout << angles.x << " | " << angles.y << " | " << angles.z << std::endl;

	mR = gsf::RPY2rotMatrix(angles,false);
	angles.x = 90.0;
	angles.y = 90.0;
	angles.z = 0.0;
	mT = gsf::RPY2rotMatrix(angles,false);
	moveit_side_functions::matrix2Dprod(mR,mT,mR);
	for(int i = 0; i < mR.size(); i++)
	{
		for(int j = 0; j < mR[i].size(); j++)
		{
			std::cout << mR[i][j] << " ";
		}
		std::cout << std::endl;
	}
	std::cout << std::endl;

	counter_conv = gsf::rotMatrix2RPY(mR,false);
	std::cout << counter_conv.x << " | " << counter_conv.y << " | " << counter_conv.z << std::endl;
	mR = gsf::RPY2rotMatrix(counter_conv,false);
	for(int i = 0; i < mR.size(); i++)
	{
		for(int j = 0; j < mR[i].size(); j++)
		{
			std::cout << mR[i][j] << " ";
		}
		std::cout << std::endl;
	}

	std::cout << std::endl;

	counter_conv = gsf::rotMatrix2RPY(mR,false);
	std::cout << counter_conv.x << " | " << counter_conv.y << " | " << counter_conv.z << std::endl;
	mR = gsf::RPY2rotMatrix(counter_conv,false);
	for(int i = 0; i < mR.size(); i++)
	{
		for(int j = 0; j < mR[i].size(); j++)
		{
			std::cout << mR[i][j] << " ";
		}
		std::cout << std::endl;
	}

	std::cout << std::endl;

*/

	std::vector<moveit_msgs::CollisionObject> obj_list = of::GetMoveitSceneObjects(planning_scene_interface);
	of::removeObject(planning_scene_interface, obj_list);

	double cell_size = 0.1;
	std::vector< std::vector< std::vector< double > > > cellmap =  plf::mapCell_generator(4.0, 4.0, 4.0, cell_size);

	std::cout << cellmap.size() << " | " << cellmap[0].size() << " | " << cellmap[0][0].size() << std::endl;

	moveit_msgs::CollisionObject obj2pub = mbf::collision_obj_generator("blabla", msf::makeVector3(1.04,1.05,0.0), 0.199);
	of::addObject(planning_scene_interface, obj2pub);

	obj_list = of::GetMoveitSceneObjects(planning_scene_interface);

	/*
	std::cout << obj_list[0].primitive_poses[0].position.x << std::endl;
	std::cout << obj_list[0].primitive_poses[0].position.y << std::endl;
	std::cout << obj_list[0].primitive_poses[0].position.z << std::endl;
    */

	std::vector<moveit_msgs::CollisionObject> obj_new_list;

	double temp_size[3];

	plf::mapCell_objsInsertion(cellmap,cell_size,obj_list);
	plf::mapCell_saturator(cellmap);
	geometry_msgs::Vector3 temp_vector;
	std::string temp_str;
	char temp_char= 'a';

	std::cout << cellmap.size()/2 << " | " << cellmap[0].size()/2 << " | " << cellmap[0][0].size()/2 << std::endl;

	std::vector<geometry_msgs::Vector3> blocks_coord = plf::mapCell_extractor(cellmap,cell_size);

	for(int i = 0; i< blocks_coord.size(); i++)
	{
		temp_char++;
		temp_str = "box_" + temp_char;
		moveit_msgs::CollisionObject temp_obj = mbf::collision_obj_generator(temp_str, blocks_coord[i], msf::makeVector3(0.0,0.0,0.0), msf::makeVector3(cell_size,cell_size,cell_size));

		ros::Duration(0.1).sleep();
		of::addObject(planning_scene_interface, temp_obj);
	}



	/*
	std::vector<moveit_msgs::CollisionObject> obj2pub = brs::Baxter_arm_objcts_occupancy("right");
	of::addObject(planning_scene_interface, obj2pub);

	std::vector<std::string> obj_id = planning_scene_interface.getKnownObjectNames();
	std::map<std::string, moveit_msgs::CollisionObject > objpres = planning_scene_interface.getObjects();

	for(int i = 0; i < obj_id.size(); i++)
	{
		std::cout << objpres[obj_id[i]].id << std::endl;
	}

	/*
	std::vector<double> one_arm_init;
	//one_arm_init = mbf::getOneArmJointPositionFromTopic("right",node_handle);

	bool success;

	one_arm_init = thf::get_ititial_joint_value();
	move_group.setJointValueTarget(one_arm_init);
	move_group.move();
	//position check
	std::cout << msf::VectorEquivalence_theshold(one_arm_init, mbf::getBothArmJointPositionFromTopic(node_handle), 0.05) << std::endl;

	one_arm_init = thf::get_shake_ititial_joint_value_up();
	move_group.setJointValueTarget(one_arm_init);
	move_group.move();
	//position check
	std::cout << msf::VectorEquivalence_theshold(one_arm_init, mbf::getBothArmJointPositionFromTopic(node_handle), 0.05) << std::endl;

	one_arm_init = thf::get_ititial_joint_value();
	move_group.setJointValueTarget(one_arm_init);
	move_group.move();
	//position check
	std::cout << msf::VectorEquivalence_theshold(one_arm_init, mbf::getBothArmJointPositionFromTopic(node_handle), 0.05) << std::endl;

	one_arm_init = thf::get_shake_ititial_joint_value_left_right();
	move_group.setJointValueTarget(one_arm_init);
	move_group.move();
	//position check
	std::cout << msf::VectorEquivalence_theshold(one_arm_init, mbf::getBothArmJointPositionFromTopic(node_handle), 0.05) << std::endl;

	one_arm_init = thf::get_ititial_joint_value();
	move_group.setJointValueTarget(one_arm_init);
	move_group.move();
	//position check
	std::cout << msf::VectorEquivalence_theshold(one_arm_init, mbf::getBothArmJointPositionFromTopic(node_handle), 0.05) << std::endl;


	std::vector<geometry_msgs::Pose> poses2use = thf::read_pose(thf::file_handler_r("/set_pose"));
	moveit::planning_interface::MoveGroup move_group2("right_arm");
	move_group2.setPoseTarget(poses2use[0]);
	move_group2.move();

	std::cout << msf::PoseEquivalence_XYZ(poses2use[0], mbf::getEePoseFromTopic("right", node_handle), 0.02) << std::endl;
	std::cout << msf::PoseEquivalence_Quat(poses2use[0], mbf::getEePoseFromTopic("right", node_handle), 0.05) << std::endl;
	std::cout << msf::PoseEquivalence_theshold(poses2use[0], mbf::getEePoseFromTopic("right", node_handle), 0.02, 0.05) << std::endl;

	move_group2.setPoseTarget(poses2use[2]);
	move_group2.move();

	std::cout << msf::PoseEquivalence_XYZ(poses2use[2], mbf::getEePoseFromTopic("right", node_handle), 0.02) << std::endl;
	std::cout << msf::PoseEquivalence_Quat(poses2use[2], mbf::getEePoseFromTopic("right", node_handle), 0.05) << std::endl;
	std::cout << msf::PoseEquivalence_theshold(poses2use[2], mbf::getEePoseFromTopic("right", node_handle), 0.02, 0.05) << std::endl;


	/*std::vector<double> one_arm;
	one_arm.resize(7);
	one_arm[0] = 0.0;
	one_arm[1] = -0.5;
	one_arm[2] = 0.5;
	one_arm[3] = 0.5;
	one_arm[4] = 0.1;
	one_arm[5] = 1.0;
	one_arm[6] = 2.4;

	move_group.setJointValueTarget(one_arm);

	move_group.move();

	move_group.setJointValueTarget(one_arm_init);

	move_group.move();*/

	//thf::write_seq_pose(node_handle);

	//while(ros::ok()){
	//vacuum lib test
	//thf::vacuum_on_off(node_handle, true);
	//ros::Duration(3.0).sleep();
	//thf::vacuum_on_off(node_handle, false);
	//}


	/*std_msgs::Bool boolT;
	ros::Publisher vacuum_pub = node_handle.advertise<std_msgs::Bool>("/vacuum",1);
	ros::Duration(3.0).sleep();


	boolT.data = true;
	vacuum_pub.publish(boolT);
	ros::Duration(3.0).sleep();
	boolT.data = false;
	vacuum_pub.publish(boolT);*/

	//std_msgs::ColorRGBA color_test = obj_functions::get_color_code("HITE");
	//std::cout << color_test.r << " " << color_test.g << " " << color_test.b << " " << color_test.a << std::endl;

/*
	std::vector<std::string> n;
	n.push_back("abc");
	n.push_back("def");
	for(int i = 0; i < n.size(); i++)
		std::cout << n[i] << std::endl;
	n.clear();
	n.push_back("blubla");
	for(int i = 0; i < n.size(); i++)
	{
		std::cout << n[i] << std::endl;
		std::string temp_string;
		temp_string = n[i];
		std::cout << n[i][2] << std::endl;}


	moveit_msgs::CollisionObject obj_temp;
	std::vector<moveit_msgs::CollisionObject> obj_list =  moveit_basics_functions::readCollisionObj(thf::file_handler_r());

	moveit::planning_interface::PlanningSceneInterface interface;
	obj_functions::addObject(interface,obj_list,"WHITE");

	obj_list =  moveit_basics_functions::readCollisionObj(thf::file_handler_r());
	obj_functions::addObject(interface,obj_list,"BLUE2");

	obj_list =  moveit_basics_functions::readCollisionObj(thf::file_handler_r());
	obj_functions::addObject(interface,obj_list,"GRAY");




	std::cout << obj_list.size() << std::endl;

	for (int i = 0; i < obj_list.size(); i++){
		obj_temp = obj_list[i];
		std::cout << obj_temp.id << " " << obj_temp.primitive_poses[0].position.x << " " << obj_temp.primitive_poses[0].position.y << " " << obj_temp.primitive_poses[0].position.z << std::endl;

	}

	//thf::write_seq_pose(node_handle);

	//std::cout << ptr << std::endl;

	//geometry_msgs::Pose pose_temp;

	//pose_temp = moveit_basics_functions::getEePoseFromTopic("left",node_handle);

	//std::cout << "right" << "\t" << pose_temp.position.x << "\t" << pose_temp.position.y << std::endl;

	//thf::write_seq_pose(node_handle);

	//std::vector<geometry_msgs::Pose> set_pose = thf::read_pose(thf::file_handler_r());

	//std::cout << set_pose.size() << std::endl;
	//success = thf::yes_not(temp_string);
	//thf::file_handler_w();
/*


	std::cout << temp_string.size() << std::endl;
	for(int i = 0; i < temp_string.size(); i++)
		string_char[i] = temp_string[i];
    //string_char[temp_string.size()] = '/0';

	std::ifstream file(&string_char);
	success = file.good();

	//success = thesis_functions::existFile(string_char[]);
	 * */
	//std::cout << success << std::endl;

	ros::shutdown();
	return 0;
}
