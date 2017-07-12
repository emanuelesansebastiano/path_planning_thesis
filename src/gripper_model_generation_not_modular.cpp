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
	ros::init(argc, argv, "gripper_pub_monica");
	ros::NodeHandle node_handle("~");

	ros::AsyncSpinner spinner(1);
	spinner.start();

	//publisher
	ros::Publisher urdf_pub_ = node_handle.advertise<baxter_core_msgs::URDFConfiguration>("/robot/urdf",1);


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
	    <link name=\"attachment\">\
	      <inertial>\
	       <origin rpy=\"0.0 0.0 0.0\" xyz=\"0.0 0.0 0.0\"/>\
	       <mass value=\"0.2\"/>\
	       <inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/>\
	     </inertial>\
	      <visual>\
		<origin xyz=\"0.0 0.0 -0.03\"/>\
		<geometry>\
		  <box size=\"0.03 0.03 0.065\" />\
		</geometry>\
		<material name=\"holidaygreen\">\
		   <color rgba=\"0.0 1 0 1\" />\
		</material>\
	      </visual>\
	      <collision>\
		<origin xyz=\"0.0 0.0 -0.03\"/>\
		<geometry>\
		  <box size=\"0.03 0.03 0.065\" />\
		</geometry>\
	      </collision>\
	    </link>\
	    <link name=\"left_gripper\">\
	      <inertial>\
	       <origin rpy=\"0.0 0.0 0.0\" xyz=\"0.0 0.0 0.0\"/>\
	       <mass value=\"0.8\"/>\
	       <inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/>\
	     </inertial>\
	      <visual>\
		<origin xyz=\"0.0 0.0 -0.11\"/>\
		<geometry>\
		  <box size=\"0.03 0.03 0.24\" />\
		</geometry>\
		<material name=\"holidaygreen\">\
		   <color rgba=\"0.0 1 0 1\" />\
		</material>\
	      </visual>\
	      <collision>\
		<origin xyz=\"0.0 0.0 -0.1\"/>\
		<geometry>\
		  <box size=\"0.03 0.03 0.22\" />\
		</geometry>\
	      </collision>\
	    </link>\
	    <joint name=\"left_gripper_base\" type=\"fixed\">\
		<origin rpy=\"0 0 3.14159265359\" xyz=\"0.0 0.0 0.075\"/>\
		<parent link=\"left_hand\"/>\
		<child link=\"attachment\"/>\
	    </joint>\
	    <joint name=\"left_endpoint\" type=\"fixed\">\
		<origin rpy=\"0 -1.5707963267 0\" xyz=\"-0.257 0.0 0.0\"/>\
		<parent link=\"attachment\"/>\
		<child link=\"left_gripper\"/>\
	    </joint>\
	  </robot>";
		//for daniels 0.10 -0.25, majd 0.08 -0.24
		baxter_core_msgs::URDFConfiguration gripper_config;
		gripper_config.time = ros::Time::now();
		gripper_config.link = "left_hand";
		gripper_config.joint = "left_gripper_base";
		gripper_config.urdf = gripper_urdf.str();

		ROS_INFO_STREAM("Published gripper urdf");
		urdf_pub_.publish(gripper_config);

		std::stringstream vacuum_urdf;
		vacuum_urdf <<	"<robot name=\"vacuum\">\
		     <link name=\"weight_sensor\">\
		       <inertial>\
			<origin rpy=\"0.0 0.0 0.0\" xyz=\"0.0 0.0 0.0\"/>\
			<mass value=\"0.2\"/>\
			<inertia ixx=\"0.01\" ixy=\"0\" ixz=\"0\" iyy=\"0.01\" iyz=\"0\" izz=\"0.01\"/>\
		      </inertial>\
		       <visual>\
			 <origin xyz=\"0.0 0.0 -0.045\"/>\
			 <geometry>\
			   <box size=\"0.03 0.03 0.105\" />\
			 </geometry>\
		<material name=\"holidaygreen\">\
		   <color rgba=\"0.0 1 0 1\" />\
			 </material>\
		       </visual>\
		       <collision>\
			 <origin xyz=\"0.0 0.0 -0.045\"/>\
			 <geometry>\
			   <box size=\"0.03 0.03 0.105\" />\
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
			 <origin xyz=\"0.0 0.0 -0.13 \"/>\
			 <geometry>\
			   <box size=\"0.03 0.03 0.26 \"/>\
			 </geometry>\
		<material name=\"holidaygreen\">\
		   <color rgba=\"0.0 1 0 1\" />\
			 </material>\
		       </visual>\
		       <collision>\
			 <origin xyz=\"0.0 0.0 -0.13 \"/>\
			 <geometry>\
			   <box size=\"0.03 0.03 0.26 \"/>\
			 </geometry>\
		       </collision>\
		     </link>\
		     <joint name=\"right_gripper_base\" type=\"fixed\">\
			 <origin rpy=\"0 0 3.14159265359\" xyz=\"-0.03 0.0 0.105\"/>\
			 <parent link=\"right_hand\"/>\
			 <child link=\"weight_sensor\"/>\
		     </joint>\
		     <joint name=\"right_endpoint\" type=\"fixed\">\
			 <origin rpy=\"0 -1.5707963267 0\" xyz=\"-0.28 0.005 0.005\"/>\
			 <parent link=\"weight_sensor\"/>\
			 <child link=\"right_gripper\"/>\
		     </joint>\
		   </robot>";
		//was -0.30 -0.01 0.0
		baxter_core_msgs::URDFConfiguration config;
		config.time = ros::Time::now();
		config.link = "right_hand";
		config.joint = "right_gripper_base";
		config.urdf = vacuum_urdf.str();

		ROS_INFO_STREAM("Published vacuum urdf");
		urdf_pub_.publish(config);

	}

	
	ros::shutdown();
	return 0;
}

