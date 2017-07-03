/* Author: Emanuele Sansebastiano
   Desc: This program modify the Baxter's gripper in the real robot
   Usage: Just run this program, modify the values of the boxes if required
*/

#include <ros/ros.h>
#include <baxter_core_msgs/URDFConfiguration.h>


#define pi 3.1415926535897931
#define security_val 0.005
#define origine_shift -0.025 //the gripper do not start from the centre of the wrist
#define zero_val 0.0

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gripper_pub");
	ros::NodeHandle node_handle("~");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	//publisher
	ros::Publisher urdf_pub = node_handle.advertise<baxter_core_msgs::URDFConfiguration>("/robot/urdf",1);


	//Left hand (gripper)
	//  gripper size
	double part1sl[] = {0.05, 0.13, 0.13};
	double part2sl[] = {0.04, 0.12, 0.32};

	//  gripper boxes size
	double box1sl[] = {part1sl[0], part1sl[1], part1sl[2]};
	double box2sl[] = {part2sl[0], part2sl[1], part2sl[2]-part1sl[0]};

	//gripper base definition (orientation_position)
	double bg_oril[] = {zero_val, zero_val, zero_val};
	double bg_posl[] = {origine_shift, zero_val, box1sl[2] + security_val};

	//box1 origin definition
	double box1ol[] = {zero_val, zero_val, -box1sl[2]/2};

	//end-point definition (orientation_position)
	double ep_oril[] = {zero_val, -pi/2, zero_val};
	double ep_posl[] = {box2sl[2] + box1sl[0]/2 + security_val, zero_val, -box2sl[0]/2};

	//box1 origin definition
	double box2ol[] = {zero_val, zero_val, box2sl[2]/2};

	//right hand (vacuum)
	//  gripper size
	double part1sr[] = {0.04, 0.04, 0.11};
	double part2sr[] = {0.04, 0.04, 0.31};

	//  gripper boxes size
	double box1sr[] = {part1sr[0], part1sr[1], part1sr[2]};
	double box2sr[] = {part2sr[0], part2sr[1], part2sr[2]-part1sr[0]};

	//gripper base definition (orientation_position)
	double bg_orir[] = {zero_val, zero_val, zero_val};
	double bg_posr[] = {origine_shift, zero_val, box1sr[2] - security_val};

	//box1 origin definition
	double box1or[] = {zero_val, zero_val, -box1sr[2]/2};

	//end-point definition (orientation_position)
	double ep_orir[] = {zero_val, -pi/2, zero_val};
	double ep_posr[] = {box2sr[2] + box1sr[0]/2 + security_val, zero_val, -box2sr[0]/2};

	//box1 origin definition
	double box2or[] = {zero_val, zero_val, box2sr[2]/2};


	while(ros::ok())
	{
		ros::Duration(1.0).sleep();

		std::stringstream gripper_urdf;
		gripper_urdf << "<robot name=\"gripper\">\
	      <link name=\"left_gripper_base\">\
	        <inertial>\
	          <origin rpy=\"0.0 0.0 0.0\" xyz=\"0.0 0.0 0.0\"/>\
	          <mass value=\"0.2\"/>\
	          <inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/>\
	        </inertial>\
	        <visual>\
	          <origin xyz=\"" << box1ol[0] << " " << box1ol[1] << " " << box1ol[2] << "\"/>\
	          <geometry>\
	            <box size=\"" << box1sl[0] << " " << box1sl[1] << " " << box1sl[2] << "\" />\
	          </geometry>\
	          <material name=\"White\">\
	            <color rgba=\"1.0 1.0 1.0 1.0\" />\
	          </material>\
	        </visual>\
	        <collision>\
	          <origin xyz=\"" << box1ol[0] << " " << box1ol[1] << " " << box1ol[2] << "\"/>\
	          <geometry>\
	            <box size=\"" << box1sl[0] << " " << box1sl[1] << " " << box1sl[2] << "\" />\
	          </geometry>\
	        </collision>\
	      </link>\
	      <link name=\"left_gripper\">\
	        <inertial>\
	         <origin rpy=\"0.0 0.0 0.0\" xyz=\"0.0 0.0 0.0\"/>\
	         <mass value=\"0.2\"/>\
	         <inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/>\
	       </inertial>\
	       <visual>\
	         <origin xyz=\"" << box2ol[0] << " " << box2ol[1] << " " << box2ol[2] << "\"/>\
	         <geometry>\
	           <box size=\"" << box2sl[0] << " " << box2sl[1] << " " << box2sl[2] << "\" />\
	         </geometry>\
	          <material name=\"White\">\
	            <color rgba=\"1.0 1.0 1.0 1.0\" />\
	         </material>\
	       </visual>\
	       <collision>\
	         <origin xyz=\"" << box2ol[0] << " " << box2ol[1] << " " << box2ol[2] << "\"/>\
	         <geometry>\
	           <box size=\"" << box2sl[0] << " " << box2sl[1] << " " << box2sl[2] << "\" />\
	         </geometry>\
	       </collision>\
	     </link>\
	     <joint name=\"left_gripper_base\" type=\"fixed\">\
	       <origin rpy=\"" << bg_oril[0] << " " << bg_oril[1] << " " << bg_oril[2] << "\" xyz=\"" << bg_posl[0] << " " << bg_posl[1] << " " << bg_posl[2] << "\"/>\
	       <parent link=\"left_hand\"/>\
	       <child link=\"left_gripper_base\"/>\
	     </joint>\
	     <joint name=\"left_endpoint\" type=\"fixed\">\
	       <origin rpy=\"" << ep_oril[0] << " " << ep_oril[1] << " " << ep_oril[2] << "\" xyz=\"" << ep_posl[0] << " " << ep_posl[1] << " " << ep_posl[2] << "\"/>\
	       <parent link=\"left_gripper_base\"/>\
	       <child link=\"left_gripper\"/>\
	     </joint>\
	   </robot>";

		baxter_core_msgs::URDFConfiguration gripper_config;
		gripper_config.time = ros::Time::now();
		gripper_config.link = "left_hand";
		gripper_config.joint = "left_gripper_base";
		gripper_config.urdf = gripper_urdf.str();

		ROS_INFO_STREAM("Published gripper urdf");
		urdf_pub.publish(gripper_config);

		std::stringstream vacuum_urdf;
		vacuum_urdf <<	"<robot name=\"vacuum\">\
	      <link name=\"right_gripper_base\">\
	        <inertial>\
	          <origin rpy=\"0.0 0.0 0.0\" xyz=\"0.0 0.0 0.0\"/>\
	          <mass value=\"0.2\"/>\
	          <inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/>\
	        </inertial>\
	        <visual>\
	          <origin xyz=\"" << box1or[0] << " " << box1or[1] << " " << box1or[2] << "\"/>\
	          <geometry>\
	            <box size=\"" << box1sr[0] << " " << box1sr[1] << " " << box1sr[2] << "\" />\
	          </geometry>\
	          <material name=\"Blue\">\
	            <color rgba=\"0.0 0.0 0.8 1.0\" />\
	          </material>\
	        </visual>\
	        <collision>\
	          <origin xyz=\"" << box1or[0] << " " << box1or[1] << " " << box1or[2] << "\"/>\
	          <geometry>\
	            <box size=\"" << box1sr[0] << " " << box1sr[1] << " " << box1sr[2] << "\" />\
	          </geometry>\
	        </collision>\
	      </link>\
	      <link name=\"right_gripper\">\
	        <inertial>\
	         <origin rpy=\"0.0 0.0 0.0\" xyz=\"0.0 0.0 0.0\"/>\
	         <mass value=\"0.2\"/>\
	         <inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/>\
	       </inertial>\
	       <visual>\
	         <origin xyz=\"" << box2or[0] << " " << box2or[1] << " " << box2or[2] << "\"/>\
	         <geometry>\
	           <box size=\"" << box2sr[0] << " " << box2sr[1] << " " << box2sr[2] << "\" />\
	         </geometry>\
	          <material name=\"Blue\">\
	            <color rgba=\"0.0 0.0 0.8 1.0\" />\
	         </material>\
	       </visual>\
	       <collision>\
	         <origin xyz=\"" << box2or[0] << " " << box2or[1] << " " << box2or[2] << "\"/>\
	         <geometry>\
	           <box size=\"" << box2sr[0] << " " << box2sr[1] << " " << box2sr[2] << "\" />\
	         </geometry>\
	       </collision>\
	     </link>\
	     <joint name=\"right_gripper_base\" type=\"fixed\">\
	       <origin rpy=\"" << bg_orir[0] << " " << bg_orir[1] << " " << bg_orir[2] << "\" xyz=\"" << bg_posr[0] << " " << bg_posr[1] << " " << bg_posr[2] << "\"/>\
	       <parent link=\"right_hand\"/>\
	       <child link=\"right_gripper_base\"/>\
	     </joint>\
	     <joint name=\"right_endpoint\" type=\"fixed\">\
	       <origin rpy=\"" << ep_orir[0] << " " << ep_orir[1] << " " << ep_orir[2] << "\" xyz=\"" << ep_posr[0] << " " << ep_posr[1] << " " << ep_posr[2] << "\"/>\
	       <parent link=\"right_gripper_base\"/>\
	       <child link=\"right_gripper\"/>\
	     </joint>\
	   </robot>";

		baxter_core_msgs::URDFConfiguration config;
		config.time = ros::Time::now();
		config.link = "right_hand";
		config.joint = "right_gripper_base";
		config.urdf = vacuum_urdf.str();

		ROS_INFO_STREAM("Published vacuum urdf");
		urdf_pub.publish(config);

	}
	ros::shutdown();
	return 0;
}

