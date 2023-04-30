#include <rclcpp/rclcpp.hpp> 
#include <navigation/navigation.hpp>
#include <iostream>

rclcpp::Node::SharedPtr mapNode;
rclcpp::Node::SharedPtr laserNode;
bool first = true;
nav_msgs::msg::OccupancyGrid::SharedPtr first_map;
nav_msgs::msg::OccupancyGrid::SharedPtr current_map;
geometry_msgs::msg::Pose::SharedPtr goal_pos;

// Callback function
void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // loop through the map.......................
  if (first){ //if first time get original map
	   first_map = msg;
	   first = false;
   }
  else{
    //std::vector<int> indices;
    //for (i = 0; i < msg.data.size(); i++){
    //  if(first_map->data[i] != msg->data[i]){
    //    indices.push_back(i);
    //  }
    //}
    int x,y,diff, arrLoc;
    RCLCPP_INFO(mapNode->get_logger(), "Looking Around");
    for (int w = 0; w < msg->info.height; ++w){
  		for ( int h=0; h < msg->info.height; h++){
  			// find x/y for the big map
  			x = (w* msg->info.resolution)+(msg->info.resolution /2) - 10 ;
  			y = (h* msg->info.resolution)+(msg->info.resolution /2 ) - 10;
			//calculate position in 1 by ~~~ array
  			arrLoc = (y * 384) + x -10;
  			//check for differences between current and first map
  			diff = first_map->data[arrLoc] - msg->data[arrLoc];
  			if (diff > 50){
				std::cout << "somethings fishy here" << x <<", " << y << std::endl;
			  }  			
  		}
  		  RCLCPP_INFO(mapNode->get_logger(), "Moving on");
    }
  }
  
}

void laserCallback(const sensor_msgs::msg::LaserScan msg) {
  //Convert laserscan data to a occupancy grid
  x_pos = goal_pos->position.x
  y_pos = goal_pos->position.y
  for i, dist in enumerate(msg.ranges):
    if dist < msg.range_max:
        x = int((x_pos + dist * math.cos(msg.angle_min + i * msg.angle_increment)) / current_map.info.resolution)
        y = int((y_pos + dist * math.sin(msg.angle_min + i * msg.angle_increment)) / current_map.info.resolution)
        if x >= 0 and x < current_map.info.width and y >= 0 and y < current_map.info.height:
            current_map.data[y * current_map.info.width + x] = 100
  mappub->publish(current_map);
}

//def mapping(x_pos_t , y_pos_t, theta, scan_msg, map_msg):
// convert laser scans to occupancy grid
//for i, dist in enumerate(scan_msg.ranges):
//    if dist < scan_msg.range_max:
//        x = int((x_pos_t + dist * math.cos(scan_msg.angle_min + i * scan_msg.angle_increment)) / map_msg.info.resolution)
//        y = int((y_pos_t + dist * math.sin(scan_msg.angle_min + i * scan_msg.angle_increment)) / map_msg.info.resolution)
//        if x >= 0 and xr < map_msg.info.width and y >= 0 and y < map_msg.info.height:
//            map_msg.data[y * map_msg.info.width + x] = 100
//return map_msg


int main(int argc,char **argv) {
 
  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose
  
  // Create instance of a node
  mapNode = rclcpp::Node::make_shared("mapnode");
  laserNode = rclcpp::Node::make_shared("lasernode");
  
  // Subscribe to the laser data
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  auto lasersub = laserNode->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, &laserCallback);
  mappub = laserNode->create_publisher<nav_msgs::msg::OccupancyGrid>("laserVals", 10)
  	
  // subscribe to topic "stringm" an register the callback function
  //laser_sub = nodeh->create_subscription<nav_msgs::msg::OccupancyGrid>
  //                                      ("mapVals",10,&callback);
                                        
   //subscriber to MAP
   auto mapsub = mapNode->create_subscription<nav_msgs::msg::OccupancyGrid>("global_costmap", 10, &mapCallback);

  //initialize map
  rclcpp::spin_some(mapNode)

   auto map = mapNode->create_subscription<nav_msgs::msg::OccupancyGrid>("laserVals", 10, &mapCallback);

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
  	
  	RCLCPP_INFO(mapNode->get_logger(), "Spinning");
    
    //After reaching new location spin laserNode to process scan data to map then spin mapNode to compare current map to first initialized
    rclcpp::spin_some(laserNode)
    rclcpp::spin_some(mapNode)
    
  	while ( ! navigator.IsTaskComplete() ) {}
  	RCLCPP_INFO(mapNode->get_logger(), "First Iteration Done");
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

    //After reaching new location spin laserNode to process scan data to map then spin mapNode to compare current map to first initialized
    rclcpp::spin_some(laserNode)
    rclcpp::spin_some(mapNode)
  	
  	
  	while ( ! navigator.IsTaskComplete() ) {}
   }
   


  
  //MRTP CH7 has the laser scan info
  // Use Global cost map, it will show abnormalities on the map
  // Type is mav_msgs/msg/Ocupancy grid
  // Check the array then check the next instance and look for differences, move toward the differences till we're at the pole
  // Next goal is to get map data and analyze it for differences
  
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}
