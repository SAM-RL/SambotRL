#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sambot_navigation/goals_publisher.h>

namespace sambot_navigation
{

const int GoalsPublisher::kNumRobotsOne = 1;
const int GoalsPublisher::kNumRobotsFour = 4;

GoalsPublisher::GoalsPublisher(
    const std::string& in_center_goal_topic, int num_robots, int grid_cells_from_center,
    float grid_cell_size) :
      num_robots_(num_robots),
      grid_cells_from_center_(grid_cells_from_center),
      grid_cell_size_(grid_cell_size)
{
    ros::NodeHandle nh;

    // Setup subscriber
    center_goal_sub_ = nh.subscribe(in_center_goal_topic, 10, &GoalsPublisher::Callback, this);

    // Setup publisher(s)
    if (num_robots_ == kNumRobotsOne)
    {
        sambot1_goal_pub_ =
            nh.advertise<geometry_msgs::PoseStamped>("/sambot1/move_base_simple/goal", 5);
    }
    else if (num_robots_ == kNumRobotsFour)
    {
        sambot1_goal_pub_ =
            nh.advertise<geometry_msgs::PoseStamped>("/sambot1/move_base_simple/goal", 5);
        sambot2_goal_pub_ =
            nh.advertise<geometry_msgs::PoseStamped>("/sambot2/move_base_simple/goal", 5);
        sambot3_goal_pub_ =
            nh.advertise<geometry_msgs::PoseStamped>("/sambot3/move_base_simple/goal", 5);
        sambot4_goal_pub_ =
            nh.advertise<geometry_msgs::PoseStamped>("/sambot4/move_base_simple/goal", 5);
    }
    else
    {
        throw std::runtime_error(
            "Currently, only setups with 1 or 4 robots is supported. Exiting.");
    }

    // Set distance from center
    distance_from_center_ = static_cast<float>(grid_cells_from_center_) * grid_cell_size_;
}

GoalsPublisher::~GoalsPublisher() = default;

void GoalsPublisher::Callback(const geometry_msgs::PoseStampedConstPtr& center_goal_pose)
{
    // Create TF message for center goal
    geometry_msgs::TransformStamped center_tf_msg;
    center_tf_msg.header.stamp = center_goal_pose->header.stamp;
    center_tf_msg.header.frame_id = "map";
    center_tf_msg.child_frame_id = "center_goal";

    center_tf_msg.transform.translation.x = center_goal_pose->pose.position.x;
    center_tf_msg.transform.translation.y = center_goal_pose->pose.position.y;
    center_tf_msg.transform.translation.z = 0.0;

    center_tf_msg.transform.rotation.x = center_goal_pose->pose.orientation.x;
    center_tf_msg.transform.rotation.y = center_goal_pose->pose.orientation.y;
    center_tf_msg.transform.rotation.z = center_goal_pose->pose.orientation.z;
    center_tf_msg.transform.rotation.w = center_goal_pose->pose.orientation.w;

    tf_br_.sendTransform(center_tf_msg);
    curr_center_goal_tf_ = center_tf_msg;

    // Generate transforms for robots' goals
    // Sample transform message
    geometry_msgs::TransformStamped sample_msg;
    sample_msg.header.stamp = ros::Time::now();
    sample_msg.transform.translation.x = 0;
    sample_msg.transform.translation.y = 0;
    sample_msg.transform.translation.z = 0;

    sample_msg.transform.rotation.x = 0; // Don't forget to fix this
    sample_msg.transform.rotation.y = 0;
    sample_msg.transform.rotation.z = 1;
    sample_msg.transform.rotation.w = 0;

    if (num_robots_ == kNumRobotsOne)
    {
        // Sambot1 should be `distance_from_center_` in the X-direction
        geometry_msgs::TransformStamped sambot1_tf_msg = sample_msg;
        sambot1_tf_msg.header.frame_id = "center_goal";
        sambot1_tf_msg.child_frame_id = "sambot1_goal";
        sambot1_tf_msg.transform.translation.x = distance_from_center_;
        sambot1_tf_msg.transform.translation.y = distance_from_center_;

        tf_br_.sendTransform(sambot1_tf_msg);
        curr_sambot1_goal_tf_ = sambot1_tf_msg;

        // Create PoseStamped message
        geometry_msgs::PoseStamped sambot1_pose_in_center_goal_frame =
            GetPoseFromTransform(sambot1_tf_msg);

        // Publish goal
        sambot1_goal_pub_.publish(sambot1_pose_in_center_goal_frame);
    }
    else if (num_robots_ == kNumRobotsFour)
    {
        // Create robot tf messages
        // Sambot1 should be `distance_from_center_` in the X-direction
        // geometry_msgs::TransformStamped sambot1_tf_msg = sample_msg;
        // sambot1_tf_msg.header.frame_id = "center_goal";
        // sambot1_tf_msg.child_frame_id = "sambot1_goal";
        // sambot1_tf_msg.transform.translation.x = distance_from_center_;
        // sambot1_tf_msg.transform.translation.y = distance_from_center_;

        // // Sambot2 should be `distance_from_center_` in the Y-direction
        // geometry_msgs::TransformStamped sambot2_tf_msg = sample_msg;
        // sambot2_tf_msg.header.frame_id = "center_goal";
        // sambot2_tf_msg.child_frame_id = "sambot2_goal";
        // sambot2_tf_msg.transform.translation.x = -distance_from_center_;
        // sambot2_tf_msg.transform.translation.y = distance_from_center_;

        // // Sambot3 should be `-distance_from_center_` in the X-direction
        // geometry_msgs::TransformStamped sambot3_tf_msg = sample_msg;
        // sambot3_tf_msg.header.frame_id = "center_goal";
        // sambot3_tf_msg.child_frame_id = "sambot3_goal";
        // sambot3_tf_msg.transform.translation.x = -distance_from_center_;
        // sambot3_tf_msg.transform.translation.y = -distance_from_center_;

        // // Sambot4 should be `-distance_from_center_` in the Y-direction
        // geometry_msgs::TransformStamped sambot4_tf_msg = sample_msg;
        // sambot4_tf_msg.header.frame_id = "center_goal";
        // sambot4_tf_msg.child_frame_id = "sambot4_goal";
        // sambot4_tf_msg.transform.translation.x = distance_from_center_;
        // sambot4_tf_msg.transform.translation.y = -distance_from_center_;
        geometry_msgs::TransformStamped sambot1_tf_msg = sample_msg;
        sambot1_tf_msg.header.frame_id = "map";
        sambot1_tf_msg.child_frame_id = "sambot1_goal";
        sambot1_tf_msg.transform.translation.x =
            center_tf_msg.transform.translation.x + distance_from_center_;
        sambot1_tf_msg.transform.translation.y =
            center_tf_msg.transform.translation.y + distance_from_center_;

        // Sambot2 should be `distance_from_center_` in the Y-direction
        geometry_msgs::TransformStamped sambot2_tf_msg = sample_msg;
        sambot2_tf_msg.header.frame_id = "map";
        sambot2_tf_msg.child_frame_id = "sambot2_goal";
        sambot2_tf_msg.transform.translation.x =
            center_tf_msg.transform.translation.x - distance_from_center_;
        sambot2_tf_msg.transform.translation.y =
            center_tf_msg.transform.translation.y + distance_from_center_;

        // Sambot3 should be `-distance_from_center_` in the X-direction
        geometry_msgs::TransformStamped sambot3_tf_msg = sample_msg;
        sambot3_tf_msg.header.frame_id = "map";
        sambot3_tf_msg.child_frame_id = "sambot3_goal";
        sambot3_tf_msg.transform.translation.x =
            center_tf_msg.transform.translation.x - distance_from_center_;
        sambot3_tf_msg.transform.translation.y =
            center_tf_msg.transform.translation.y - distance_from_center_;

        // Sambot4 should be `-distance_from_center_` in the Y-direction
        geometry_msgs::TransformStamped sambot4_tf_msg = sample_msg;
        sambot4_tf_msg.header.frame_id = "map";
        sambot4_tf_msg.child_frame_id = "sambot4_goal";
        sambot4_tf_msg.transform.translation.x =
            center_tf_msg.transform.translation.x + distance_from_center_;
        sambot4_tf_msg.transform.translation.y =
            center_tf_msg.transform.translation.y - distance_from_center_;

        tf_br_.sendTransform(sambot1_tf_msg);
        tf_br_.sendTransform(sambot2_tf_msg);
        tf_br_.sendTransform(sambot3_tf_msg);
        tf_br_.sendTransform(sambot4_tf_msg);

        curr_sambot1_goal_tf_ = sambot1_tf_msg;
        curr_sambot2_goal_tf_ = sambot2_tf_msg;
        curr_sambot3_goal_tf_ = sambot3_tf_msg;
        curr_sambot4_goal_tf_ = sambot4_tf_msg;

        // Create PoseStamped messages
        geometry_msgs::PoseStamped sambot1_pose_in_center_goal_frame =
            GetPoseFromTransform(sambot1_tf_msg);

        geometry_msgs::PoseStamped sambot2_pose_in_center_goal_frame =
            GetPoseFromTransform(sambot2_tf_msg);

        geometry_msgs::PoseStamped sambot3_pose_in_center_goal_frame =
            GetPoseFromTransform(sambot3_tf_msg);

        geometry_msgs::PoseStamped sambot4_pose_in_center_goal_frame =
            GetPoseFromTransform(sambot4_tf_msg);

        // Publish goals
        sambot1_goal_pub_.publish(sambot1_pose_in_center_goal_frame);
        sambot2_goal_pub_.publish(sambot2_pose_in_center_goal_frame);
        sambot3_goal_pub_.publish(sambot3_pose_in_center_goal_frame);
        sambot4_goal_pub_.publish(sambot4_pose_in_center_goal_frame);
    }
}

geometry_msgs::PoseStamped
GoalsPublisher::GetPoseFromTransform(const geometry_msgs::TransformStamped& tf)
{
    geometry_msgs::PoseStamped pose;

    pose.header.stamp = tf.header.stamp;
    pose.header.frame_id = tf.header.frame_id;

    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;

    pose.pose.orientation.x = tf.transform.rotation.x;
    pose.pose.orientation.y = tf.transform.rotation.y;
    pose.pose.orientation.z = tf.transform.rotation.z;
    pose.pose.orientation.w = tf.transform.rotation.w;

    return pose;
}

}  // namespace sambot_navigation

int main(int argc, char** argv)
{
    ros::init(argc, argv, "formation_center_tf_broadcaster");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string in_center_goal_topic = "";
    int num_robots = 1;
    float grid_cell_size = 0.1;
    int grid_cells_from_center = 5;

    int bad_params = 0;

    bad_params += !pnh.getParam("in_center_goal_topic", in_center_goal_topic);
    bad_params += !pnh.getParam("num_robots", num_robots);
    bad_params += !pnh.getParam("grid_cell_size", grid_cell_size);
    bad_params += !pnh.getParam("grid_cells_from_center", grid_cells_from_center);

    if (bad_params > 0)
    {
        std::cout << "One or more params not set! Exiting." << std::endl;
        return 1;
    }

    sambot_navigation::GoalsPublisher fctb(
        in_center_goal_topic, num_robots, grid_cells_from_center, grid_cell_size);

    ros::spin();
    return 0;
}
