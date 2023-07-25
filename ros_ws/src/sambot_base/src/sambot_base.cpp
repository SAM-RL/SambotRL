#include <ros/ros.h>
#include <sambot_base/sambot_hw_interface.h>
#include <controller_manager/controller_manager.h>
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "sambot_hw_interface");
    ros::NodeHandle nh;
    
    sambot_base::SamBotHWInterface samBot(nh);
 
    controller_manager::ControllerManager cm(&samBot);
    
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::Time prev_time = ros::Time::now();
    ros::Rate rate(10.0);
    rate.sleep();

    while (ros::ok())
    {
        const ros::Time     time   = ros::Time::now();
        const ros::Duration period = time - prev_time;
        prev_time = time;

        samBot.read(time, period);    
        cm.update(time, period);        
        samBot.write(time, period);
        
        rate.sleep();
    }
    return 0;
}
