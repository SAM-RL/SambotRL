// Lidar Publisher Node.

#include "ros/ros.h"
#include "ydlidar/CYdLidar.h"
#include "sensor_msgs/LaserScan.h"
#include <iostream>
#include <signal.h>
#include <string>
#include <vector>

std::vector<float> split(const std::string& s, char delim)
{
    std::vector<float> elems;
    std::stringstream ss(s);
    std::string number;
    while (std::getline(ss, number, delim))
    {
        elems.push_back(atof(number.c_str()));
    }
    return elems;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Lidar_publisher");

    fflush(stdout);

    std::string port;
    std::string frame_id;
    bool resolution_fixed;
    bool auto_reconnect;
    double angle_max, angle_min;
    result_t op_result;
    std::string list;
    std::vector<float> ignore_array;
    double max_range, min_range;

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("port", port, "/dev/ydlidar");
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("resolution_fixed", resolution_fixed, "true");
    nh_private.param<bool>("auto_reconnect", auto_reconnect, "true");
    nh_private.param<double>("angle_max", angle_max, 180);
    nh_private.param<double>("angle_min", angle_min, -180);
    nh_private.param<double>("range_max", max_range, 10.0);
    nh_private.param<double>("range_min", min_range, 0.1);
    nh_private.param<std::string>("ignore_array", list, "");
    nh_private.param<std::string>("ignore_array", list, "");

    ignore_array = split(list, ',');
    if (ignore_array.size() % 2)
    {
        ROS_ERROR_STREAM("ignore array is odd need be even");
    }

    for (uint16_t i = 0; i < ignore_array.size(); i++)
    {
        if (ignore_array[i] < -180 && ignore_array[i] > 180)
        {
            ROS_ERROR_STREAM("ignore array should be between 0 and 360");
        }
    }

    sambot_driver::CYdLidar laser;
    if (angle_max < angle_min)
    {
        double temp = angle_max;
        angle_max = angle_min;
        angle_min = temp;
    }

    // ROS_INFO("[Y] Now YDLIDAR ROS SDK VERSION:%s .......", ROSVerision);
    laser.setSerialPort(port);
    laser.setMaxRange(max_range);
    laser.setMinRange(min_range);
    laser.setMaxAngle(angle_max);
    laser.setMinAngle(angle_min);
    laser.setFixedResolution(resolution_fixed);
    laser.setAutoReconnect(auto_reconnect);
    laser.setIgnoreArray(ignore_array);
    bool ret = laser.initialize();

    if (ret)
    {
        ret = laser.turnOn();
        if (!ret)
        {
            ROS_ERROR("Failed to start scan mode!!!");
        }
    }
    else
    {
        ROS_ERROR("Error initializing YDLIDAR Comms and Status!!!");
    }
    ros::Rate rate(20);

    while (ret && ros::ok())
    {
        bool hardError;
        LaserScan scan;
        if (laser.doProcessSimple(scan, hardError))
        {
            sensor_msgs::LaserScan scan_msg;
            ros::Time start_scan_time;
            start_scan_time.sec = scan.system_time_stamp / 1000000000ul;
            start_scan_time.nsec = scan.system_time_stamp % 1000000000ul;
            scan_msg.header.stamp = start_scan_time;
            scan_msg.header.frame_id = frame_id;
            scan_msg.angle_min = (scan.config.min_angle);
            scan_msg.angle_max = (scan.config.max_angle);
            scan_msg.angle_increment = (scan.config.angle_increment);
            scan_msg.scan_time = scan.config.scan_time;
            scan_msg.time_increment = scan.config.time_increment;
            scan_msg.range_min = (scan.config.min_range);
            scan_msg.range_max = (scan.config.max_range);

            scan_msg.ranges = scan.ranges;
            scan_msg.intensities = scan.intensities;
            scan_pub.publish(scan_msg);
        }
        rate.sleep();
        ros::spinOnce();
    }

    laser.turnOff();
    printf("Lidar Publishing is stopping .......\n");
    laser.disconnecting();
    return 0;
}
