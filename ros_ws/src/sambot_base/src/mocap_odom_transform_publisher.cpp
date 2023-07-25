#include <ros/ros.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

std::string chassis_tf_name[4] = { "sambot1/chassis", "sambot2/chassis", "sambot3/chassis", "sambot4/chassis" };
std::string odom_tf_name[4] = { "sambot1/odom", "sambot2/odom", "sambot3/odom", "sambot4/odom" };
geometry_msgs::TransformStamped static_transformStamped;
  
int main(int argc, char** argv){
  ros::init(argc, argv, "mocap_odom_transform_publisher");

  ros::NodeHandle node;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  ros::Rate rate(10.0);
  geometry_msgs::TransformStamped sourceTransformStamped[4];
  int i = 0;
  
  geometry_msgs::TransformStamped targetTransformStamped;
  
  while (i<4){

    try{
      sourceTransformStamped[i] = tfBuffer.lookupTransform("map", chassis_tf_name[i],
                               ros::Time(0));
      i++;
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    rate.sleep();
  }
  

  for (int i=0; i<4; i++) {
	targetTransformStamped.header.stamp = ros::Time::now();
	targetTransformStamped.child_frame_id = odom_tf_name[i];
	targetTransformStamped.header.frame_id = "map";
	targetTransformStamped.transform = sourceTransformStamped[i].transform;
	static_broadcaster.sendTransform(targetTransformStamped);
    }
  
  ros::spin();
  
  return 0;
};

