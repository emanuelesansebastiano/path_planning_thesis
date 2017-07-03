 /*Author: Emanuele Sansebastiano */

// my libs
#include <path_planning_thesis/lib_path_planning.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "vacuum_activator");
	ros::NodeHandle node_handle("~");

	ros::AsyncSpinner spinner(1);

	spinner.start();

	namespace thf = thesis_functions;

	bool on_off;
	char in_char;
	
	on_off = true;
	thf::vacuum_on_off(node_handle, on_off);

	while (in_char != 'Q' && in_char != 'q')
	{
		std::cout << std::endl << "Do not enter 'ctrl+c' or 'ctrl+z' to stop the vacuum, because it will stop the program but not the vacuum!" << std::endl;
		std::cout << "If you want to stop the vacuum enter 'Q'" << std::endl;
		std::cin >> in_char;
		if(in_char != 'Q' && in_char != 'q')
			std::cout << "Warning: An invalid input has been inserted!" << std::endl;
	}

	on_off = false;
	thf::vacuum_on_off(node_handle, on_off);

	ros::shutdown();
	return 0;
}
