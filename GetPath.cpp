
#include <rclcpp/rclcpp.hpp> 
#include <navigation/navigation.hpp>


tf2::Quaternion q;
double theta;

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
  goal_pos->position.x = 2;
  goal_pos->position.y = 1;
  goal_pos->orientation.w = 1;
  // move to new pose
  navigator.GoToPose(goal_pos);
  while ( ! navigator.IsTaskComplete() ) {//wait
    
  }
  goal_pos->position.x = 2;
  goal_pos->position.y = -1;
  goal_pos->orientation.w = 1;
  navigator.GoToPose(goal_pos);
  // move to new pose
  while ( ! navigator.IsTaskComplete() ) { //wait
    
  }
  // backup of 0.15 m (deafult distance)
  navigator.Backup();
  while ( ! navigator.IsTaskComplete() ) {
    
  }

 std::cout << "Printing Path: " << endl;

  auto path = nav.get_path(goal);   //return pointer to msg/Path arr or null
  // path is an array of poses, poses are vectors
  if (path != NULL){ 	//print array if not null
      for (i=0; i < path.poses.size(); i++){ 	
  	RCLCPP_INFO_STREAM(nav->get_logger()," X-Coordinate: "<<path.poses[i].pose.position.x);
  	RCLCPP_INFO_STREAM(nav->get_logger()," X-Coordinate: " <<path.poses[i].pose.position.y);
  	tf2::convert(msg->pose.pose.orientation,q) //make quaternion
          tf2::Matrix3x3 m(q);   //make matrix to get theta
          m.getRPY(roll,pitch,theta); //get theta from quaternion
          
  	RCLCPP_INFO_STREAM(nav->get_logger(),"Theta: "<< theta);
  	// RCLCPP_INFO_STREAM(nav->get_logger(), "msgs" << var
  	// just info uses c-printing, stream uses cpp
      }
  }
  
  rclcpp::shutdown(); // shutdown ROS
  return 0;
}

/*
Template from Stefano Carpin
@https://github.com/stefanocarpin/MRTP 
*/

