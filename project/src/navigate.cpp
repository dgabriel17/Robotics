#include <rclcpp/rclcpp.hpp> 
#include <navigation/navigation.hpp>
#include <iostream>

rclcpp::Node::SharedPtr nodeh;
bool first = true;
nav_msgs::msg::OccupancyGrid::SharedPtr first_map;


// Callback function for map
void callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // loop through the map.......................
  if (first){ //if first time get original map
	   first_map = msg;
	   first = false;
   }
  else{
    int x,y,diff, arrLoc;
    RCLCPP_INFO(nodeh->get_logger(), "Looking Around ---------------------------");
    for (unsigned int w = 0; w < msg->info.height; ++w){
  		for (unsigned int h=0; h < msg->info.height; h++){
  			// find x/y for the big map
  			x = (w* msg->info.resolution)+ (msg->info.resolution /2) - 10 ;
  			y = (h* msg->info.resolution) + ( msg->info.resolution /2 ) - 10;
			//calculate position in 1 by ~~~ array
  			arrLoc = (y * 384) + x -10;
  			//check for differences between current and first map
  			diff = first_map->data[arrLoc] - msg->data[arrLoc];
  			if (diff > 50){
				std::cout << "somethings fishy here" << x <<", " << y << std::endl;
			}  			
  			
  		}
  		  RCLCPP_INFO(nodeh->get_logger(), "Moving on");
    }
  }
  
}


int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose
  
  // Create instance of a node
  nodeh = rclcpp::Node::make_shared("navigate");
                                   
   //subscriber to MAP
   auto firstmap = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>("map", 1, &callback);

   rclcpp::spin_some(nodeh);

   auto sub = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>("global_costmap/costmap", 1, &callback);

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
  	
    // wait for messages and process them                
    rclcpp::spin_some(nodeh); 
    
  	while ( ! navigator.IsTaskComplete() ){
  	}

   }  
   
   RCLCPP_INFO(nodeh->get_logger(), "---------Done with first loop---------");
   rclcpp::spin_some(nodeh); 
   
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

    rclcpp::spin_some(nodeh);
  	
  	while ( ! navigator.IsTaskComplete() )  {
  	}
  	
   }

  //MRTP CH7 has the laser scan info
  // Use Global cost map, it will show abnormalities on the map
  // Type is mav_msgs/msg/Ocupancy grid
  // Check the array then check the next instance and look for differences, move toward the differences till we're at the pole
  // Next goal is to get map data and analyze it for differences
  
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}
