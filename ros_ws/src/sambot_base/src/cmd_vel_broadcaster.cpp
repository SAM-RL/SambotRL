#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub1,pub2,pub3,pub4;

void velocityCallback(const geometry_msgs::Twist& msg){
    //geometry_msgs::Twist msg;
    pub1.publish(msg);
    pub2.publish(msg);
    pub3.publish(msg);
    pub4.publish(msg);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "cmd_vel_broardcaster");
  ros::NodeHandle nh;
  
  pub1 = nh.advertise<geometry_msgs::Twist>("sambot1/mobile_base_controller/cmd_vel", 100);
  pub2 = nh.advertise<geometry_msgs::Twist>("sambot2/mobile_base_controller/cmd_vel", 100);
  pub3 = nh.advertise<geometry_msgs::Twist>("sambot3/mobile_base_controller/cmd_vel", 100);
  pub4 = nh.advertise<geometry_msgs::Twist>("sambot4/mobile_base_controller/cmd_vel", 100);

  ros::Subscriber sub = nh.subscribe("mobile_base_controller/cmd_vel", 100, &velocityCallback);
  ros::Rate rate(20);
  
  while(ros::ok()){
     rate.sleep();
     ros::spinOnce();      //Notice this
  } 
}
