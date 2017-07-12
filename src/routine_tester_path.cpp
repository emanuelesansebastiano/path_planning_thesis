 /*Author: Emanuele Sansebastiano */

// my libs
#include <path_planning_thesis/lib_path_planning.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "routine_tester_path");
	ros::NodeHandle node_handle("~");

	ros::AsyncSpinner spinner(1);

	spinner.start();

	namespace msf = moveit_side_functions;
	namespace mbf = moveit_basics_functions;
	namespace of = obj_functions;
	namespace thf = thesis_functions;

	//VARIEBLES
	//****************************************
	bool success, g_reach;
	int iteration_n, pose_n, cin_int, pose_save_counter;
	char selected_ch, l_r;
	std::string l_r_b, temp_question, cin_str, directory_str;
	std::ofstream file_pose, file_joint;

	std::vector<double> joints_value, joints_value_up, plan_move_time;
	std::vector<double> double_vec_temp1, double_vec_temp2;
	std::vector<geometry_msgs::Pose> poses2use;
	moveit::planning_interface::MoveGroup::Plan thesis_plan;
	moveit::planning_interface::PlanningSceneInterface plan_interface;
	std::vector<moveit_msgs::CollisionObject> obj_list;
	std::vector<geometry_msgs::Pose> motion_pose_list;
	std::vector<std::vector<double>> motion_joint_pos_list;
	moveit::planning_interface::MoveGroup::Plan my_plan;

	const std::string error_string = "Warning: An invalid input has been inserted!";
	const std::string comment_str_pose = "right-left-both\tplanner_name\titeration\tnum_goal_pose\tmotion_success\tnum_saved_position\tpose_values:p.x_p.y_p.z_o.w_o.x_o.y_o.z";
	const std::string comment_str_joint = "right-left-both\tplanner_name\titeration\tnum_goal_pose\tmotion_success\tnum_saved_position\tjoint_values_starting_from_shoulder2end-effector";
	std::string sides_loc[] = {"left_arm", "right_arm", "both_arms"};
	char sides_char[] = {'l', 'r', 'b', '%'};
	std::vector<std::string> planner_names = mbf::getOmplPlannerList();
	int test_times = 5;
	double pos_threhold = 0.05, ori_threshold = 0.05, joint_threshold = 0.05;
	double max_plan_time = 120.0;
	double pause_time = 0.5;

	//position initial position
	moveit::planning_interface::MoveGroup move_g_init(sides_loc[2]);
	joints_value = thf::get_ititial_joint_value();
	joints_value_up = thf::get_shake_ititial_joint_value_up();

	ros::Duration(2.0).sleep();
	success = false; while(!success)
	{	//move_g_init.setJointValueTarget(joints_value); //moveit direct command
		of::setJointValuesTarget(move_g_init,joints_value); //thesis function
		plan_move_time = of::plan_execute_safely_t(move_g_init);
		success = true;
		ros::Duration(pause_time).sleep();
		//position check
		success = msf::VectorEquivalence_theshold(joints_value, mbf::getBothArmJointPositionFromTopic(node_handle), joint_threshold);
		std::cout << "Joint angle position reached: " << success << std::endl;
	}


	// USER SETTINGS!
	//****************************************
	// scene obj question
	temp_question = "Do you want to load an object scene from a file?";
	success = true;
	while(success)
	{
		success = thf::yes_not(temp_question);
		if(success)
		{
			obj_list = mbf::readCollisionObj(thf::file_handler_r("/scenes"));
			of::addObject(plan_interface, obj_list);
		}
	}

	// set of end effector pose question
	std::cout << "A file containing a set of poses must be read..." << std::endl;
	poses2use = thf::read_pose(thf::file_handler_r("/poses"));

	// arm used question
	success = false;
	while(!success)
	{
		std::cout << "Which arm are you going to use for this experiment?" << std::endl;
		std::cout << "Enter 'R' for the right one" << std::endl;
		std::cout << "Enter 'L' for the left one" << std::endl;
		std::cout << "Enter 'B' for both arms together" << std::endl;

		std::cin >> selected_ch;
		if(selected_ch == 'l' || selected_ch == 'L'){
			success = true;
			l_r_b = sides_loc[0];
			l_r = sides_char[0];
		}else if(selected_ch == 'r' || selected_ch == 'R'){
			success = true;
			l_r_b = sides_loc[1];
			l_r = sides_char[1];
		}else if(selected_ch == 'b' || selected_ch == 'B'){
			success = true;
			l_r_b = sides_loc[2];
			l_r = sides_char[2];
		}else{
			std::cout << error_string << std::endl;
		}
	}

	// file_name pose
	std:: cout << "Pose file selection...\nGive a name like: 'pose_file'_pose" << std::endl;
	directory_str = thf::file_handler_w("/results/path");
	//change to char string
	char * dir_file_char = new char[directory_str.size() + 1];
	std::copy(directory_str.begin(), directory_str.end(), dir_file_char);
	dir_file_char[directory_str.size()] = '\0'; // don't forget the terminating 0
	//open file
	file_pose.open(dir_file_char, std::ios::app);

	// file_name joint
	std:: cout << "Joint position file selection...\nGive a name like: 'pose_file'_joint" << std::endl;
	directory_str = thf::file_handler_w("/results/path");
	//change to char string
	dir_file_char = new char[directory_str.size() + 1];
	std::copy(directory_str.begin(), directory_str.end(), dir_file_char);
	dir_file_char[directory_str.size()] = '\0'; // don't forget the terminating 0
	//open file
	file_joint.open(dir_file_char, std::ios::app);


	// PROTOCOL STARTS!
	//****************************************
	//moveit initialization
	moveit::planning_interface::MoveGroup move_group(l_r_b);
	// to check
	move_g_init.setPlanningTime(max_plan_time); //moveit direct command
	//of::setPlanningTime(move_group, max_plan_time); //side_moveit_pkg function

	//first line comment insertion
	temp_question = "Do you want to write comment lines?\nInserting the 'pose_file', 'arms used' and 'scene files' is suggested";
	success = true;
	while(success)
	{
		success = thf::yes_not(temp_question);
		while(success)
		{
			std::cout << "Enter the comment line..." << std::endl;
		  	std::cin.ignore();
		  	std::getline(std::cin,  cin_str);
		  	file_pose << sides_char[3] << " " << cin_str << " " << sides_char[3] << std::endl;
		  	file_joint << sides_char[3] << " " << cin_str << " " << sides_char[3] << std::endl;
		  	std::cout << "Comment inserted successfully" << std::endl;
		  	//another comment
		  	temp_question = "Do you want to write another comment line?";
		  	success = thf::yes_not(temp_question);
		}
	}
	//file structure comment
  	file_pose << sides_char[3] << " " << comment_str_pose << " " << sides_char[3] << std::endl;
  	file_joint << sides_char[3] << " " << comment_str_joint << " " << sides_char[3] << std::endl;

	//for every planner
	for(int i = 0; i < planner_names.size(); i++)
	{
		//Path planner selection
		if(i == 0)
		{
			success = false;
			while(!success)
			{
			  	temp_question = "Do you want to select a path planner from which you want to start?";
			  	success = !thf::yes_not(temp_question);
			  	if(!success)
			  	{
			  		while(!success)
			  		{
			  			for(int pp = 0; pp < planner_names.size(); pp++)
			  				std::cout << "enter '" << pp << "' to select the path planner called: '" << planner_names[pp] << "'" << std::endl;
			  			std::cin >> cin_int;
			  			if(cin_int >= 0 && cin_int < planner_names.size())
			  			{	success = true;
			  				i = cin_int;
			  			}else{
			  				std::cout << error_string << std::endl;
			  			}
			  		}
			  	}
			}
		}

		if(!ros::ok())
		{	std::cout << "Program killed!" << std::endl;
			goto end_program;}

		//pause to ask the user to change the planner
		std::cout << std::endl << std::endl;
		std::cout << "The set planner is: '" << planner_names[i] << "'" << std::endl;
		std::cout << std::endl << std::endl;

		// change planner motion check
		// force to go to the starting position
		success = false; while(!success)
		{	//move_g_init.setJointValueTarget(joints_value); //moveit direct command
			of::setJointValuesTarget(move_g_init,joints_value_up); //thesis function
			plan_move_time = of::plan_execute_safely_t(move_g_init);
			success = true;
			ros::Duration(pause_time).sleep();
			//position check
			success = msf::VectorEquivalence_theshold(joints_value_up, mbf::getBothArmJointPositionFromTopic(node_handle), joint_threshold);
			std::cout << "Joint angle position reached: " << success << std::endl;
		}
		success = false; while(!success)
		{	//move_g_init.setJointValueTarget(joints_value); //moveit direct command
			of::setJointValuesTarget(move_g_init,joints_value); //thesis function
			plan_move_time = of::plan_execute_safely_t(move_g_init);
			success = true;
			ros::Duration(pause_time).sleep();
			//position check
			success = msf::VectorEquivalence_theshold(joints_value, mbf::getBothArmJointPositionFromTopic(node_handle), joint_threshold);
			std::cout << "Joint angle position reached: " << success << std::endl;
		}

		//planner setting
		move_g_init.setPlannerId(planner_names[i]); //moveit direct command
		//of::setPlanner(move_group, planner_names[i]); //side_moveit_pkg function

		//for every test repetition planned
		iteration_n = 0;
		for(int y = 0; y < test_times ; y++)
		{
			if(!ros::ok())
			{	std::cout << "Program killed!" << std::endl;
				goto end_program;}

			iteration_n = y;

			//for every pose in the set
			pose_n = 0;
			for(int p = 0; p < poses2use.size(); p++)
			{
				if(!ros::ok())
				{	std::cout << "Program killed!" << std::endl;
					goto end_program;}

				if(l_r_b != sides_loc[2])
				{
					//move_group.setPoseTarget(poses2use[p]); //moveit direct command
					of::setEePoseTarget(move_group, poses2use[p]); //side_moveit_pkg function
				}else{
					//move_group.setPoseTarget(poses2use[p], "left_gripper"); //moveit direct command
					of::setEePoseTarget(move_group, poses2use[p], "left"); //side_moveit_pkg function
					p++;
					//move_group.setPoseTarget(poses2use[p], "right_gripper"); //moveit direct command
					of::setEePoseTarget(move_group, poses2use[p], "right"); //side_moveit_pkg function
				}

				//initialization variables to save in the file
				pose_save_counter = 0;
				motion_pose_list.clear();
				motion_joint_pos_list.clear();

				// effective motion
				//PLANINFICATION
				success = move_group.plan(my_plan); //side_moveit_pkg function
				if(success)
				{
					//save end-effetor pose and joint positions
					if(l_r_b != sides_loc[2])
					{
						if(l_r_b == sides_loc[0]){
							motion_pose_list.push_back(mbf::getEePoseFromTopic("left", node_handle));
							motion_joint_pos_list.push_back(mbf::getOneArmJointPositionFromTopic("left", node_handle));
						}else{
							motion_pose_list.push_back(mbf::getEePoseFromTopic("right", node_handle));
							motion_joint_pos_list.push_back(mbf::getOneArmJointPositionFromTopic("right", node_handle));
						}
					}else{
						motion_pose_list.push_back(mbf::getEePoseFromTopic("left", node_handle));
						motion_pose_list.push_back(mbf::getEePoseFromTopic("right", node_handle));
						motion_joint_pos_list.push_back(mbf::getBothArmJointPositionFromTopic(node_handle));
					}
					pose_save_counter++;

					//EXECUTION OF THE PATH PLANNED
					move_group.asyncExecute(my_plan);

					//save end-effetor pose and joint positions
					if(l_r_b != sides_loc[2])
					{
						if(l_r_b == sides_loc[0]){
							motion_pose_list.push_back(mbf::getEePoseFromTopic("left", node_handle));
							motion_joint_pos_list.push_back(mbf::getOneArmJointPositionFromTopic("left", node_handle));
						}else{
							motion_pose_list.push_back(mbf::getEePoseFromTopic("right", node_handle));
							motion_joint_pos_list.push_back(mbf::getOneArmJointPositionFromTopic("right", node_handle));
						}
					}else{
						motion_pose_list.push_back(mbf::getEePoseFromTopic("left", node_handle));
						motion_pose_list.push_back(mbf::getEePoseFromTopic("right", node_handle));
						motion_joint_pos_list.push_back(mbf::getBothArmJointPositionFromTopic(node_handle));
					}
					pose_save_counter++;

					//while joint position is not the same as the previous one
					double_vec_temp1 = motion_joint_pos_list[motion_joint_pos_list.size()-2];
					double_vec_temp2 = motion_joint_pos_list[motion_joint_pos_list.size()-1];
					while(!msf::VectorEquivalence_theshold(double_vec_temp1, double_vec_temp2, 0.001))
					{
						//save end-effetor pose and joint positions
						if(l_r_b != sides_loc[2])
						{
							if(l_r_b == sides_loc[0]){
								motion_pose_list.push_back(mbf::getEePoseFromTopic("left", node_handle));
								motion_joint_pos_list.push_back(mbf::getOneArmJointPositionFromTopic("left", node_handle));
							}else{
								motion_pose_list.push_back(mbf::getEePoseFromTopic("right", node_handle));
								motion_joint_pos_list.push_back(mbf::getOneArmJointPositionFromTopic("right", node_handle));
							}
						}else{
							motion_pose_list.push_back(mbf::getEePoseFromTopic("left", node_handle));
							motion_pose_list.push_back(mbf::getEePoseFromTopic("right", node_handle));
							motion_joint_pos_list.push_back(mbf::getBothArmJointPositionFromTopic(node_handle));
						}
						pose_save_counter++;

						// while condition definition
						double_vec_temp1 = motion_joint_pos_list[motion_joint_pos_list.size()-2];
						double_vec_temp2 = motion_joint_pos_list[motion_joint_pos_list.size()-1];
					}
				}
				//goal reached check
				if(l_r_b != sides_loc[2])
				{
					if(l_r_b == sides_loc[0]){
						g_reach = msf::PoseEquivalence_theshold(poses2use[p], mbf::getEePoseFromTopic("left", node_handle), pos_threhold, ori_threshold);
					}else{
						g_reach = msf::PoseEquivalence_theshold(poses2use[p], mbf::getEePoseFromTopic("right", node_handle), pos_threhold, ori_threshold);}
				}else{
					//left check
					g_reach = msf::PoseEquivalence_theshold(poses2use[p-1], mbf::getEePoseFromTopic("left", node_handle), pos_threhold, ori_threshold);
					if(g_reach){
						//right check
						g_reach = msf::PoseEquivalence_theshold(poses2use[p], mbf::getEePoseFromTopic("right", node_handle), pos_threhold, ori_threshold);}
				}


				//screen output
				std::cout << "Planner " << i << ": " << planner_names[i] << std:: endl;
				std::cout << "Iteration: " << iteration_n << std::endl;
				std::cout << "Number saved trajectory waypoints: " << pose_save_counter << std::endl;
				std::cout << "Goal reached: " << g_reach << std::endl;

				//save the spanned poses
				if(l_r_b != sides_loc[2])
				{	//one arm
					file_pose << l_r << "\t" << planner_names[i] << "\t" << iteration_n << "\t" << pose_n << "\t" << g_reach << "\t" << pose_save_counter << "\t";
					for(int pose = 0; pose < motion_pose_list.size(); pose++)
						file_pose << motion_pose_list[pose].position.x << "\t" << motion_pose_list[pose].position.y << "\t" << motion_pose_list[pose].position.z << "\t" << motion_pose_list[pose].orientation.w << "\t" << motion_pose_list[pose].orientation.x << "\t" << motion_pose_list[pose].orientation.y << "\t" << motion_pose_list[pose].orientation.z ;
					file_pose << std::endl;
				}else{ // both arms
					//left
					file_pose << "l" << "\t" << planner_names[i] << "\t" << iteration_n << "\t" << pose_n << "\t" << g_reach << "\t" << pose_save_counter << "\t";
					for(int pose = 0; pose < motion_pose_list.size(); pose += 2)
						file_pose << motion_pose_list[pose].position.x << "\t" << motion_pose_list[pose].position.y << "\t" << motion_pose_list[pose].position.z << "\t" << motion_pose_list[pose].orientation.w << "\t" << motion_pose_list[pose].orientation.x << "\t" << motion_pose_list[pose].orientation.y << "\t" << motion_pose_list[pose].orientation.z ;
					file_pose << std::endl;
					//right
					file_pose << "r" << "\t" << planner_names[i] << "\t" << iteration_n << "\t" << pose_n << "\t" << g_reach << "\t" << pose_save_counter << "\t";
					for(int pose = 1; pose < motion_pose_list.size(); pose += 2)
						file_pose << motion_pose_list[pose].position.x << "\t" << motion_pose_list[pose].position.y << "\t" << motion_pose_list[pose].position.z << "\t" << motion_pose_list[pose].orientation.w << "\t" << motion_pose_list[pose].orientation.x << "\t" << motion_pose_list[pose].orientation.y << "\t" << motion_pose_list[pose].orientation.z ;
					file_pose << std::endl;
				}
				//save the spanned joint values
				if(l_r_b != sides_loc[2])
				{	//one arm
					file_joint << l_r << "\t" << planner_names[i] << "\t" << iteration_n << "\t" << pose_n << "\t" << g_reach << "\t" << pose_save_counter;
					for(int pose = 0; pose < motion_joint_pos_list.size(); pose++)
						for(int pose_j = 0 ; pose_j < motion_joint_pos_list[pose].size(); pose_j++)
							file_joint << "\t" << motion_joint_pos_list[pose][pose_j];
					file_joint << std::endl;
				}else{ //both arms
					//left
					file_joint << "l" << "\t" << planner_names[i] << "\t" << iteration_n << "\t" << pose_n << "\t" << g_reach << "\t" << pose_save_counter;
					for(int pose = 0; pose < motion_joint_pos_list.size(); pose++)
						for(int pose_j = 0 ; pose_j < motion_joint_pos_list[pose].size()/2; pose_j++)
							file_joint << "\t" << motion_joint_pos_list[pose][pose_j];
					file_joint << std::endl;
					//right
					file_joint << "r" << "\t" << planner_names[i] << "\t" << iteration_n << "\t" << pose_n << "\t" << g_reach << "\t" << pose_save_counter;
					for(int pose = 0; pose < motion_joint_pos_list.size(); pose++)
						for(int pose_j = motion_joint_pos_list[pose].size()/2 +1 ; pose_j < motion_joint_pos_list[pose].size(); pose_j++)
							file_joint << "\t" << motion_joint_pos_list[pose][pose_j];
					file_joint << std::endl;
				}

				ros::Duration(pause_time).sleep();

				pose_n++;
			}

			// forced to go back to the starting position position
			success = false; while(!success)
			{	//move_g_init.setJointValueTarget(joints_value); //moveit direct command
				of::setJointValuesTarget(move_g_init,joints_value); //thesis function
				plan_move_time = of::plan_execute_safely_t(move_g_init);
				success = true;
				ros::Duration(pause_time).sleep();
				//position check
				success = msf::VectorEquivalence_theshold(joints_value, mbf::getBothArmJointPositionFromTopic(node_handle), joint_threshold);
				std::cout << "Joint angle position reached: " << success << std::endl;
			}
		}
	}

	// in case of emergency: PROGRAM TERMINATED
	end_program:

	move_group.stop(); //useful just for asyncronous motions
	file_pose.close();
	spinner.stop();

	//deleting section
	delete[] dir_file_char;

	ros::shutdown();
	return 0;
}
