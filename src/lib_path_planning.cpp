/* Author: Emanuele Sansebastiano
   Desc: Library to encapsulate some functions useful to accomplish the "path Planning" final project based on Baxter.
         Amazon Robotics Challenge - Universidad Jaume I (Spain) [Competitor]
*/

// this pkg
#include <path_planning_thesis/lib_path_planning.h>

namespace msf = moveit_side_functions;
namespace mbf = moveit_basics_functions;
namespace of = obj_functions;

//////////////////////////////////////////////////////////////////////////////////////
// VALUES NOT MODIFIABLE BY THE USER \\

const std::string error_string = "Warning: An invalid input has been inserted!";
//useful char codes '%' is used for the comments inside the files
//DO NOT CHANGE THIS SYMBOLS! Do it just if you are sure you fully understood this library
const char sides[] = {'l', 'r', 'b', '%'};

//////////////////////////////////////////////////////////////////////////////////////

namespace thesis_functions
{
  //global_var
  std_msgs::Bool gl_bool;

  bool yes_not(std::string question)
  {
	  bool success;
	  bool done = false;
	  char selected_ch;
	  while (!done)
	  {
		  std::cout << question << std::endl << "(Enter 'Y' for yes or 'N' for not)" << std::endl;
		  std::cin >> selected_ch;
		  if(selected_ch == 'n' || selected_ch == 'N'){
			  success = false;
			  done = true;
		  }else if(selected_ch == 'y' || selected_ch == 'Y'){
			  success = true;
			  done = true;
		  }else{
			  std::cout << error_string << std::endl;
		  }
	  }

	  return success;
  }

  bool existFile(std::string directoryName)
  {
	  bool success;

	  //existence check
	  success = boost::filesystem::exists(directoryName);

	  return success;
  }

  std::string file_handler_r(std::string sub_directory)
  {
	  bool success;
	  std::string file_str;
	  std::string question;
	  int selected_num;

	  restart:
	  // file name question
	  success = false;
	  while(!success)
	  {
		  std::cout << "How is the file you want to read (" << file_type << ") called?" << std::endl;
		  std::cin >> file_str;
		  file_str = file_str + file_type;
		  question = "Is the file called '" + sub_directory + "/" + file_str + "'?";
		  success = yes_not(question);
	  }

	  // whole directory file definition; 'script_directory_string' is editable in the .h file
	  file_str = script_directory_string + sub_directory + "/" + file_str;

	  // existence check and relative decision
	  success = false;
	  while(!success && !existFile(file_str)){
		  std::cout << "The file does not exist. What do you want to do?" << std::endl;
		  std::cout << "Enter '1' for quitting" << std::endl;
		  std::cout << "Enter '2' if you want to change the file name" << std::endl;

		  std::cin >> selected_num;
		  if(selected_num == 1){
			  //quitting
			  success = true;
			  file_str = "The file does not exist!";
			  std::cout << "The file does not exist. A default string (" << file_str << ") has been returned" << std::endl;
		  }else if(selected_num == 2){
			  //changing the file name
			  success = true;
			  goto restart;
		  }else{
			  std::cout << error_string << std::endl;
		  }
	  }

	  return file_str;
  }

  std::string file_handler_w(std::string sub_directory)
  {
	  bool success;
	  std::string file_str;
	  std::string question;
	  int selected_num;

	  restart:
	  // file name question
	  success = false;
	  while(!success)
	  {
		  std::cout << "How is the file you want to edit/generate (" << file_type << ") called?" << std::endl;
		  std::cin >> file_str;
		  file_str = file_str + file_type;
		  question = "Is the file called '" + sub_directory + "/" + file_str + "'?";
		  success = yes_not(question);
	  }

	  // whole directory file definition; 'script_directory_string' is editable in the .h file
	  file_str = script_directory_string + sub_directory + "/" + file_str;

	  // transforming the string to char* data
	  char * temp_file_char = new char[file_str.size() + 1];
	  std::copy(file_str.begin(), file_str.end(), temp_file_char);
	  temp_file_char[file_str.size()] = '\0'; // don't forget the terminating 0

	  // existence check and relative decision
	  if (existFile(file_str))
	  {
		  success = false;
		  while(!success){
			  std::cout << "The file already exists. What do you want to do?" << std::endl;
			  std::cout << "Enter '1' for writing in appendix to the file" << std::endl;
			  std::cout << "Enter '2' for overwriting the file" << std::endl;
			  std::cout << "Enter '3' if you want to change the file name" << std::endl;

			  std::cin >> selected_num;
			  if(selected_num == 1){
				  //writing in appendix
				  success = true;
			  }else if(selected_num == 2){
				  //overwriting
				  std::string quest = "Are you sure you want to overwrite this file loosing the data contained in it?";
				  success = yes_not(quest);
				  //deleting the file
				  if(success){
					  if(remove(temp_file_char) != 0)
						  perror("Error occurred deleting the file");
					  //creating the file
					  std::ofstream file(temp_file_char);
				  }
			  }else if(selected_num == 3){
				  //changing the file name
				  success = true;
				  goto restart;
			  }else{
				  std::cout << error_string << std::endl;
			  }
		  }
	  }else{
		  //generate the file
		  std::ofstream file(temp_file_char);
	  }
	  //deleting section
	  delete[] temp_file_char;

	  return file_str;
  }

  std::vector<geometry_msgs::Pose> read_pose(std::string directory_file)
  {
	  geometry_msgs::Pose temp_pose;
	  std::vector<geometry_msgs::Pose> poses2return;
	  char in_char; int num; double p_x, p_y, p_z, o_w, o_x, o_y, o_z;

	  const int side_check_size = 3;
	  char side_check[side_check_size]; int l_r_count = 0;
	  int num_check[side_check_size]; int num_count = 0;

	  //existence check
	  if(!existFile(directory_file))
	  {
		  perror("The file you want to read does not exist. A zero pose has been returned");
		  return poses2return;
	  }
	  std::ifstream file(directory_file, std::ios::in);
	  //openability check
	  if(!file.is_open()){
		  perror("The file you want to read cannot be opened. A zero pose has been returned");
		  return poses2return;
	  }else{

		  file >> in_char;
		  while(!file.eof())
		  {
			  if(in_char == sides[3])
			  {	  // read the comment until it is not closed
				  in_char = sides[3]-1; //to de-initialize the variable
				  while(in_char != sides[3])
					  {file >> in_char;}
				  //uncomment the following line to check
				  //std::cout << sides[3] << " comment line " << sides[3] << std::endl;
			  }else if(in_char == sides[0] || in_char == sides[1]){

				  //Check file (l-r): various side pose could be mixed up
				  side_check[l_r_count] = in_char; l_r_count++;
				  if(l_r_count > side_check_size-1)
				  {
					  if((side_check[0] == side_check[1] && side_check[1] != side_check[2]) || (side_check[0] != side_check[1] && side_check[1] == side_check[2]))
					  {std::cout << "warning: in the read file there is this pose sequential " << side_check[0] << ", " << side_check[1] << ", " << side_check[2] << std::endl;}
					  l_r_count = 0;
				  }

				  //number of the pose
  				  file >> num;
  				  num_check[num_count] = num; num_count++;
  				  if(num_count > side_check_size-1)
  				  {
  					  if((num_check[0] == num_check[1] && num_check[1] == num_check[2]) || num_check[0] < num_check[1]-1 || num_check[1] < num_check[2]-1)
  					  {std::cout << "warning: in the read file there a not regular pose sequence, check the poses number " << num_check[0] << ", " << num_check[1] << ", " << num_check[2] << std::endl;}
  					  num_count = 0;
  				  }

				  // position
				  file >> p_x >> p_y >> p_z;
				  //orientation
				  file >> o_w >> o_x >> o_y >> o_z;
				  temp_pose = msf::makePose(msf::makeQuat(o_w,o_x,o_y,o_z), msf::makeVector3(p_x,p_y,p_z));
				  //uncomment the following two lines to check
				  //std::cout << "pose " << in_char << " number " << num << " read correctly" << std::endl;
				  //std::cout << temp_pose.position.y << "; " << temp_pose.orientation.x << std::endl;
				  poses2return.push_back(temp_pose);
			  }
			  else{
				  std::cout << "Warning: this symbol has been found '" << in_char << "'" << std::endl;
				  std::cout << "It is not either a 'r'-'l' (right - left) pose or a '%' sentence (comments)" << std::endl;
				  std::cout << "CHECK THE FILE! This symbol has been ignored..." << std::endl;
			  }
			  file >> in_char;
			  //uncomment the following line to check
			  //std::cout << in_char << std::endl;
		  }
		  file.close();
	  }

	  return poses2return;
  }

  void write_seq_pose(ros::NodeHandle &nh)
  {
	  bool success;
	  char selected_ch;
	  char l_r_b;
	  int count_pose;
	  std::string cin_str;
	  std::string side_full;
	  geometry_msgs::Pose pose_temp;
	  std::ofstream file;
	  std::string directory_file = file_handler_w();

	  std::vector<std::string> pos_sides; pos_sides.resize(3);
	  pos_sides[0] = "left"; pos_sides[1] = "right"; pos_sides[2] = "both";

	  success = false;
	  while(!success){
		  std::cout << "What do you want to save?" << std::endl;
		  std::cout << "Enter 'R' if you want the right end_effector pose" << std::endl;
		  std::cout << "Enter 'L' if you want the left end_effector pose" << std::endl;
		  std::cout << "Enter 'B' if you want the both end_effector poses" << std::endl;

		  std::cin >> selected_ch;
		  if(selected_ch == 'l' || selected_ch == 'L'){
			  success = true;
			  l_r_b = sides[0];
			  side_full = pos_sides[0];
		  }else if(selected_ch == 'r' || selected_ch == 'R'){
			  success = true;
			  l_r_b = sides[1];
			  side_full = pos_sides[1];
		  }else if(selected_ch == 'b' || selected_ch == 'B'){
			  success = true;
			  l_r_b = sides[2];
			  side_full = pos_sides[2];
		  }else{
			  std::cout << error_string << std::endl;
		  }
	  }

	  std::cout << "The pose is going to be save in this order:" <<std::endl;
	  std::cout << "side(l-r)\tnumber\tpos.x\tpos.y\tpos.z\tori.w\tori.x\tori.y\tori.z" <<std::endl;

	  char * dir_file_char = new char[directory_file.size() + 1];
	  std::copy(directory_file.begin(), directory_file.end(), dir_file_char);
	  dir_file_char[directory_file.size()] = '\0'; // don't forget the terminating 0
	  file.open(dir_file_char, std::ios::app);

	  //check if there are other poses already written
	  std::vector<geometry_msgs::Pose> set_pose_temp = read_pose(dir_file_char);
	  count_pose = set_pose_temp.size();
	  if(count_pose > 0)
	  {
		  success = false;
		  while(!success){
			  std::cout << "What do you want to do with the pose number?" << std::endl;
			  std::cout << "Enter 'C' if you want to continue the sequence" << std::endl;
			  std::cout << "Enter 'R' if you want the restart the sequence from '0'" << std::endl;

			  std::cin >> selected_ch;
			  if(selected_ch == 'c' || selected_ch == 'C'){
				  success = true;
				  std::cout << "The first new pose will start from the number '" << count_pose << "' " << std::endl;
				  std::cout << "Writing a comment is strongly suggested..." << std::endl;
			  }else if(selected_ch == 'r' || selected_ch == 'R'){
				  success = true;
				  count_pose = 0;
				  std::cout << "The first new pose will start from the number '" << count_pose << "' " << std::endl;
			  }else{
				  std::cout << error_string << std::endl;
			  }
		  }
	  }

	  std::string help_msg;
	  help_msg = "Enter 'S' to save the current pose value (" + side_full + " end-effector)\nEnter 'C' to insert a comment in the file\nEnter 'Q' to finish this task";

	  std::cout << help_msg << std::endl;
	  while(selected_ch != 'q' && selected_ch != 'Q')
	  {
		  std::cin >> selected_ch;
		  if(selected_ch == 'q' || selected_ch == 'Q')
		  {   //quit
			  file.close();
			  std::cout << "Job done!" << std::endl;
		  }else if(selected_ch == 'c' || selected_ch == 'C'){
			  std::cout << "Type the comment you want to insert..." << std::endl;
			  std::cin.ignore();
			  std::getline(std::cin,  cin_str);
			  file << sides[3] << " " << cin_str << " " << sides[3] << "\n" << std::endl;
			  std::cout << "Comment inserted successfully" << std::endl;
			  std::cout << help_msg << std::endl;
		  }else if(selected_ch == 'h' || selected_ch =='H')
		  {	  //help message
			  std::cout << help_msg << std::endl;
		  }else if(selected_ch == 's' || selected_ch == 'S')
		  {	  // save the pose
			  if(l_r_b == sides[2])
			  {   //left
				  pose_temp = mbf::getEePoseFromTopic(pos_sides[0],nh);
				  file << sides[0] << "\t" << count_pose << "\t" << pose_temp.position.x << "\t" << pose_temp.position.y << "\t" << pose_temp.position.z << "\t" << pose_temp.orientation.w << "\t" << pose_temp.orientation.x << "\t" << pose_temp.orientation.y << "\t" << pose_temp.orientation.z << std::endl;
				  //right
				  pose_temp = mbf::getEePoseFromTopic(pos_sides[1],nh);
				  file << sides[1] << "\t" << count_pose << "\t" << pose_temp.position.x << "\t" << pose_temp.position.y << "\t" << pose_temp.position.z << "\t" << pose_temp.orientation.w << "\t" << pose_temp.orientation.x << "\t" << pose_temp.orientation.y << "\t" << pose_temp.orientation.z << "\n" << std::endl;
			  }else{
				  pose_temp = mbf::getEePoseFromTopic(side_full,nh);
				  //uncomment the following line to check the pose value
				  //std::cout << l_r_b << "\t" << pose_temp.position.x << "\t" << pose_temp.position.y << std::endl;
				  file << l_r_b << "\t" << count_pose << "\t" << pose_temp.position.x << "\t" << pose_temp.position.y << "\t" << pose_temp.position.z << "\t" << pose_temp.orientation.w << "\t" << pose_temp.orientation.x << "\t" << pose_temp.orientation.y << "\t" << pose_temp.orientation.z << "\n" << std::endl;
			  }
			  std::cout << "Pose number " << count_pose << " saved correctly" << std::endl;
			  count_pose ++;
		  }else{
			  std::cout << error_string << std::endl;
			  std::cout << "Enter 'H' for help" << std::endl;
		  }
	  }

	  //deleting section
	  delete[] dir_file_char;
  }

  std::vector<double> get_ititial_joint_value(void)
  {
	  std::vector<double> vector2return; vector2return.resize(14);
	  vector2return[0] = +1.69;
	  vector2return[1] = -0.40;
	  vector2return[2] = -1.72;
	  vector2return[3] = +0.95;
	  vector2return[4] = -0.23;
	  vector2return[5] = +1.61;
	  vector2return[6] = +1.26;
	  vector2return[7] = -vector2return[0];
	  vector2return[8] = vector2return[1];
	  vector2return[9] = -vector2return[2];
	  vector2return[10] = vector2return[3];
	  vector2return[11] = -vector2return[4];
	  vector2return[12] = vector2return[5];
	  vector2return[13] = -vector2return[6];

	  return vector2return;
  }
  std::vector<double> get_shake_ititial_joint_value_up(void)
  {
	  std::vector<double> vector2return = get_ititial_joint_value();
	  double shake_val = 0.25;
	  vector2return[1] += shake_val;
	  vector2return[8] += shake_val;

	  return vector2return;
  }
  std::vector<double> get_shake_ititial_joint_value_left_right(void)
  {
	  std::vector<double> vector2return = get_ititial_joint_value();
	  double shake_val = 0.25;
	  vector2return[0] -= shake_val;
	  vector2return[7] += shake_val;

	  return vector2return;
  }

  void callback_vacuum(std_msgs::Bool on_off){
	  gl_bool = on_off;}
  void vacuum_on_off(ros::NodeHandle &nh, bool active)
  {
	  //topic string
	  std::string topic_str = "/vacuum";

	  //variables
	  std_msgs::Bool bool2pub;
	  bool2pub.data = active;
	  double time_pub = 1.0;

	  if(msf::CheckTopicExistence(topic_str))
	  {
		  //publisher - subscriber
		  ros::Publisher vacuum_pub = nh.advertise<std_msgs::Bool>(topic_str,1);
		  ros::Subscriber vacuum_sub = nh.subscribe <std_msgs::Bool>(topic_str, 1, callback_vacuum);

		  //initialization of gl_bool
		  gl_bool.data = true;
		  std::string msg = "off";
		  if(active)
		  {
			  gl_bool.data = false;
			  msg = "on";
		  }

		  //subscribe once
		  ros::spinOnce();

		  while(bool2pub.data!= gl_bool.data)
		  {
			  ros::Duration(time_pub/2).sleep();
			  vacuum_pub.publish(bool2pub);
			  ros::spinOnce();
			  ros::Duration(time_pub/2).sleep();
		  }

		  std::cout << "Vacuum gripper " << msg << "!" << std::endl;
	  }else{
		  std::cout << "The topic '" << topic_str << "' does not exist" << std::endl;
	  }
  }


// End namespace "thesis_functions"
}


/*Code to convert a string to an array of char
char * arr_char = new char[str.size() + 1];
std::copy(str.begin(), str.end(), arr_char);
arr_char[str.size()] = '\0'; // don't forget the terminating 0
*/
