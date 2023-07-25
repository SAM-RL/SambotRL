#pragma once

#include "tf2_msgs/TFMessage.h"
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

namespace sambot_navigation
{

/**
 * Class for publishing goals for the robots to reach. These goals are sent
 * both as transforms (so that we can visualize them in the TF tree easily),
 * and also as geometry_msgs::PoseStamped messages for `move_base` to follow.
 *
 * Currently, this class only supports the "diamond" formation, i.e., the
 * robots are equal distance away from the center point of the formation in
 * the North (Sambot1), East (Sambot2), South (Sambot3), West (Sambot4)
 * directions. This class can work with 1 or 4 robots.
 */
class GoalsPublisher
{
public:
    static const int kNumRobotsOne;
    static const int kNumRobotsFour;

    /**
     * Constructor
     *
     * @param in_center_goal_topic (const std::string&): Topic to subscribe to
     * for goal locations for the formation center. These goals will be sent
     * by the RL algorithm deciding how the robots should move. To send these
     * goals manually, we can publish from the command line
     * ```
     * rostopic pub /center_goal geometry_msgs/PoseStamped <msg>
     * ```
     *
     * @param num_robots (int): Number of robots to send goal locations for.
     * Currently only 1 or 4 are supported. Providing any other number will
     * make the node crash.
     *
     * @param distance_from_center (float): The distance in meters that the
     * robots are to be positioned away from the formation center.
     */
    GoalsPublisher(
        const std::string& in_center_goal_topic, int num_robots, int grid_cells_from_center,
        float grid_cell_size);
    /**
     * Destructor
     */
    ~GoalsPublisher();

    /**
     * Callback that receives the center goal pose, computes the goal poses for
     * all the robots and publishes their transforms and goal poses.
     */
    void Callback(const geometry_msgs::PoseStampedConstPtr& center_goal_pose);

private:
    // Input Parameters
    int num_robots_ = 1;
    float grid_cell_size_ = 0.1;  // in meters
    int grid_cells_from_center_ = 5;
    float distance_from_center_;

    // Class variables
    // TODO(deepak): Use these to find correct orientations later
    geometry_msgs::TransformStamped curr_center_goal_tf_;
    geometry_msgs::TransformStamped curr_sambot1_goal_tf_;
    geometry_msgs::TransformStamped curr_sambot2_goal_tf_;
    geometry_msgs::TransformStamped curr_sambot3_goal_tf_;
    geometry_msgs::TransformStamped curr_sambot4_goal_tf_;

    // Subscriber
    ros::Subscriber center_goal_sub_;

    // Publishers
    ros::Publisher sambot1_goal_pub_;
    ros::Publisher sambot2_goal_pub_;
    ros::Publisher sambot3_goal_pub_;
    ros::Publisher sambot4_goal_pub_;

    // TF related objects
    tf2_ros::TransformBroadcaster tf_br_;

    /**
     * Converts a geometry_msgs::TransformStamped object into a
     * geometry_msgs::PoseStamped object. This is used right before we publish
     * goals for the robots.
     *
     * @param tf (const geometry_msgs::TransformStamped&): Transform message.
     * In this case, it is the robot pose as viewed from the goal center
     * reference frame.
     */
    geometry_msgs::PoseStamped GetPoseFromTransform(const geometry_msgs::TransformStamped& tf);

};  // class GoalsPublisher

}  // namespace sambot_navigation
