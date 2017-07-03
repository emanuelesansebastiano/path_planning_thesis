 /*Author: Emanuele Sansebastiano */

// my libs
#include <path_planning_thesis/lib_path_planning.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "pose_saver");
	ros::NodeHandle node_handle("~");

	ros::AsyncSpinner spinner(1);

	spinner.start();

	namespace thf = thesis_functions;

	// pose saver code
	thf::write_seq_pose(node_handle);


	ros::shutdown();
	return 0;
}
