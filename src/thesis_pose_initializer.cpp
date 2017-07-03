 /*Author: Emanuele Sansebastiano */

// my libs
#include <path_planning_thesis/lib_path_planning.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "thesis_pose_initializer");
	ros::NodeHandle node_handle("~");

	ros::AsyncSpinner spinner(1);

	spinner.start();

	namespace msf = moveit_side_functions;
	namespace mbf = moveit_basics_functions;
	namespace of = obj_functions;

	namespace thf = thesis_functions;

	//moveit initialization
	static const std::string PLANNING_GROUP = "both_arms";
	moveit::planning_interface::MoveGroup move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

	std::vector<double> one_arm_init;

	one_arm_init = thf::get_ititial_joint_value();
	move_group.setJointValueTarget(one_arm_init);
	move_group.move();


	ros::shutdown();
	return 0;
}
