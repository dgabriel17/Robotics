#include <rclcpp/rclcpp.hpp> 
#include <navigation/navigation.hpp>
#include <iostream>

//rclcpp::Node::SharedPtr nodeh;

// Callback function 
void stringCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
   print received string to the screen
  RCLCPP_INFO(nodeh->get_logger(),"Map Data: %s",msg->data);
}

int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose

  // first: it is mandatory to initialize the pose of the robot
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = -2;
  init->position.y = -0.5;
  init->orientation.w = 1;
  navigator.SetInitialPose(init);
  // wait for navigation stack to become operationale
  navigator.WaitUntilNav2Active();
  // spin in place of 90 degrees (default parameter)
  navigator.Spin();
  while ( ! navigator.IsTaskComplete() ) {
    // busy waiting for task to be completed
  }
  geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
  
  float arr_loc_x[] = {-1, 0, 1, 1.5, 1.5, 1.5, 1, 0, -1, -2};
  float arr_loc_y[] = {-2, -2, -2, -1, 0, 1, 1.75, 2, 2, 0.5};
  
  // var for the location array indecies
  int arr_index = 0;
  
  // Loop until we either visit all locations or find the goal
  while (arr_index != 10)
  {
  	goal_pos->position.x = arr_loc_x[arr_index];
  	goal_pos->position.y = arr_loc_y[arr_index];
  	goal_pos->orientation.w = 1;
  	// move to new pose
  	navigator.GoToPose(goal_pos);
  	while ( ! navigator.IsTaskComplete() ) {}
  	
  	// Increment arr_index
  	arr_index++;
  	
  	// Subscribe to the laser data
  	rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr laser_sub;
  	
  	// Create instance of a node
  	nodeh = rclcpp::Node::make_shared("mapVals");
  	
	// subscribe to topic "stringm" an register the callback function
	laser_sub = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>
                                        ("mapVals",10,&stringCallback);
                                         
        rclcpp::spin(nodeh); // wait for messages and process them
  	
  	// backup of 0.15 m (deafult distance)
  	navigator.Backup();
  	while ( ! navigator.IsTaskComplete() ) {}
   }
   
  float arr_loc_x_two[] = {-0.5, 0.5, 0.5, -0.5};
  float arr_loc_y_two[] = {0.5, 0.5, -0.5, -0.5};
  
  // var for the location array indecies
  arr_index = 0;
  
  // Loop until we either visit all locations or find the goal
  while (arr_index != 4)
  {
  	goal_pos->position.x = arr_loc_x_two[arr_index];
  	goal_pos->position.y = arr_loc_y_two[arr_index];
  	goal_pos->orientation.w = 1;
  	// move to new pose
  	navigator.GoToPose(goal_pos);
  	while ( ! navigator.IsTaskComplete() ) {}
  	
  	// Increment arr_index
  	arr_index++;
  	
  	
  	// backup of 0.15 m (deafult distance)
  	navigator.Backup();
  	while ( ! navigator.IsTaskComplete() ) {}
   }
  // complete here....
  
  //MRTP CH7 has the laser scan info
  // Use Global cost map, it will show abnormalities on the map
  // Type is mav_msgs/msg/Ocupancy grid
  // Check the array then check the next instance and look for differences, move toward the differences till we're at the pole
  // Next goal is to get map data and analyze it for differences
  
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}
