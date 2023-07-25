#include <grid_map_msgs/GridMap.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <sambot_rl/spatial_temporal_field_publisher_node.h>

namespace sambot_rl
{

SpatialTemporalFieldPublisherNode::SpatialTemporalFieldPublisherNode(
    const std::string& in_update_trigger_topic, const std::string& out_field_topic,
    const std::string& out_agent_field_topic, const std::string& initial_field_state_file,
    const SpatialTemporalDiffusionParams& diff_params, int num_rows, int num_cols,
    float grid_cell_size) :
      initial_field_state_file_(initial_field_state_file),
      dx_(diff_params.dx),
      dy_(diff_params.dy),
      vx_(diff_params.vx),
      vy_(diff_params.vy),
      dt_(diff_params.dt),
      k_(diff_params.k),
      num_rows_(num_rows),
      num_cols_(num_cols),
      grid_cell_size_(grid_cell_size)
{
    ros::NodeHandle nh;

    // Setup Grid Map publishers
    grid_map_pub_ = nh.advertise<grid_map_msgs::GridMap>(out_field_topic, 1, true);
    agent_grid_map_pub_ = nh.advertise<grid_map_msgs::GridMap>(out_agent_field_topic, 1, true);
    env_polygon_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("env_polygon", 1, true);
    agent_polygon_pub_ = nh.advertise<geometry_msgs::PolygonStamped>("agent_polygon", 1, true);
    mapping_error_pub_ = nh.advertise<std_msgs::Float32>("/mapping_error", 1, true);
    formation_center_odom_pub_ =
        nh.advertise<nav_msgs::Odometry>("/formation_center_odom", 1, true);

    // Setup Trigger subscriber
    trigger_sub_ = nh.subscribe(
        in_update_trigger_topic, 1, &SpatialTemporalFieldPublisherNode::Callback, this);

    /**
     * Setup Grid Map and its properties. We will assume that the grid map is setup at the center of
     * the `map` frame. The grid map will be a square with each cell of side length equal to
     * `grid_cell_size_`. The initial field values are loaded from a .mat file.
     */
    std::vector<std::string> layers = {"concentration"};
    spatial_temporal_field_ = std::make_shared<grid_map::GridMap>(layers);
    agent_field_ = std::make_shared<grid_map::GridMap>(layers);

    // Setup Frame IDs for both grid maps
    std::string env_map_frame_id = "env_map";
    spatial_temporal_field_->setFrameId(env_map_frame_id);

    std::string agent_map_frame_id = "agent_map";
    agent_field_->setFrameId(agent_map_frame_id);

    // Setup Geometry for both fields
    spatial_temporal_field_->setGeometry(
        grid_map::Length(
            static_cast<double>(num_rows_) * grid_cell_size_,
            static_cast<double>(num_cols_) * grid_cell_size_),
        grid_cell_size_);

    agent_field_->setGeometry(
        grid_map::Length(
            static_cast<double>(num_rows_) * grid_cell_size_,
            static_cast<double>(num_cols_) * grid_cell_size_),
        grid_cell_size_);

    // Setup static transform between "map" frame and "env_map" frame
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped map_to_env_map_transform_msg;

    map_to_env_map_transform_msg.header.stamp = ros::Time::now();
    map_to_env_map_transform_msg.header.frame_id = "map";
    map_to_env_map_transform_msg.child_frame_id = env_map_frame_id;
    // map_to_env_map_transform_msg.transform.translation.x =
    //     grid_cell_size_ * (static_cast<float>(num_cols_) / 2.0);
    // map_to_env_map_transform_msg.transform.translation.y =
        // grid_cell_size_ * (static_cast<float>(num_rows_) / 2.0);
    map_to_env_map_transform_msg.transform.translation.x = 0.0;
    map_to_env_map_transform_msg.transform.translation.y = 0.0;
    map_to_env_map_transform_msg.transform.translation.z = 0.0;
    map_to_env_map_transform_msg.transform.rotation.x = 0.0;
    map_to_env_map_transform_msg.transform.rotation.y = 0.0;
    map_to_env_map_transform_msg.transform.rotation.z = 0.0;
    map_to_env_map_transform_msg.transform.rotation.w = 1.0;

    static_broadcaster.sendTransform(map_to_env_map_transform_msg);
    std::cout << "Static transform between map and env_map sent" << std::endl;

    // Setup static transform between "map" frame and "agent_map" frame
    geometry_msgs::TransformStamped map_to_agent_map_transform_msg;

    map_to_agent_map_transform_msg.header.stamp = ros::Time::now();
    map_to_agent_map_transform_msg.header.frame_id = "map";
    map_to_agent_map_transform_msg.child_frame_id = agent_map_frame_id;
    map_to_agent_map_transform_msg.transform.translation.x =
        grid_cell_size_ * (static_cast<float>(num_cols_) / 2.0);
    map_to_agent_map_transform_msg.transform.translation.y =
        3 * grid_cell_size_ * (static_cast<float>(num_rows_) / 2.0);
    map_to_agent_map_transform_msg.transform.translation.z = 0.0;
    map_to_agent_map_transform_msg.transform.rotation.x = 0.0;
    map_to_agent_map_transform_msg.transform.rotation.y = 0.0;
    map_to_agent_map_transform_msg.transform.rotation.z = 0.0;
    map_to_agent_map_transform_msg.transform.rotation.w = 1.0;

    static_broadcaster.sendTransform(map_to_agent_map_transform_msg);
    std::cout << "Static transform between map and agent_map sent" << std::endl;

    // Setup TF listener
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    // Setup initial values inside field
    InitialFieldStateSetup();

    // Publish initial field state
    PublishCurrentField();
}

SpatialTemporalFieldPublisherNode::~SpatialTemporalFieldPublisherNode() = default;

void SpatialTemporalFieldPublisherNode::InitialFieldStateSetup()
{
    // Load grid from initial_state
    cnpy::NpyArray u_var = cnpy::npy_load(initial_field_state_file_);
    double* u_arr = u_var.data<double>();

    Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> u_mat(
        u_arr, num_rows_, num_cols_);

    /**
     * The next few lines add a second source to the field
     */

    Eigen::MatrixXf u_1 = u_mat.matrix().cast<float>();

    // Source 1 location -> Same row, move column by 13 places
    int source_1_r_shift = 0;
    int source_1_c_shift = 13; // 0
    for (int r = 0; r < num_rows_; r++)
    {
        for (int c = 0; c < num_cols_; c++)
        {
            u_1(r, c) = u_mat(
                (r - source_1_r_shift) % num_rows_,
                (((c - source_1_c_shift) % num_cols_) + num_cols_) % num_cols_);
        }
    }

    Eigen::MatrixXf rev_u = u_mat.matrix().cast<float>().reverse();

    // Create a copy that is flipped around the diagonal
    // Eigen::MatrixXf rev_u = u_1.reverse();

    // // Move the second source
    // Eigen::MatrixXf rev_u_1 = u_1;
    // int source_2_r_shift = 10;
    // int source_2_c_shift = 2;
    // for (int r = 0; r < num_rows_; r++)
    // {
    //     for (int c = 0; c < num_cols_; c++)
    //     {
    //         rev_u_1(r, c) =
    //             rev_u((r + source_2_r_shift) % num_rows_, (c + source_2_c_shift) % num_cols_);
    //     }
    // }

    Eigen::MatrixXf rev_u_1 = rev_u;
    int source_2_r_shift = 10;
    int source_2_c_shift = 2;
    for (int r = 0; r < num_rows_; r++)
    {
        for (int c = 0; c < num_cols_; c++)
        {
            rev_u_1(r, c) =
                rev_u((r + source_2_r_shift) % num_rows_, (c + source_2_c_shift) % num_cols_);
        }
    }

    // // Populate the field grid map with initialized values
    // curr_u_ = (u_1 + rev_u_1).colwise().reverse().rowwise().reverse();
    // spatial_temporal_field_->add("concentration", curr_u_);
    curr_u_ = rev_u_1 + u_1;
    spatial_temporal_field_->add("concentration", curr_u_);

    // Populate the agent grid map with zeros
    agent_curr_u_ = Eigen::MatrixXf::Zero(num_rows_, num_cols_);
    agent_field_->add("concentration", agent_curr_u_);
}

void SpatialTemporalFieldPublisherNode::CopyViewScope()
{
    // TF lookup goal locations for the formation
    geometry_msgs::TransformStamped sambot1_goal_pose;
    geometry_msgs::TransformStamped sambot2_goal_pose;
    geometry_msgs::TransformStamped sambot3_goal_pose;
    geometry_msgs::TransformStamped sambot4_goal_pose;

    try
    {
        sambot1_goal_pose = tf_buffer_.lookupTransform("map", "sambot1/chassis", ros::Time(0));
        sambot2_goal_pose = tf_buffer_.lookupTransform("map", "sambot2/chassis", ros::Time(0));
        sambot3_goal_pose = tf_buffer_.lookupTransform("map", "sambot3/chassis", ros::Time(0));
        sambot4_goal_pose = tf_buffer_.lookupTransform("map", "sambot4/chassis", ros::Time(0));
    }
    catch (tf2::TransformException& ex)
    {
        std::cout << ex.what() << std::endl;
        return;
    }

    // Form polygons
    // Environment
    float sambot1_x = sambot1_goal_pose.transform.translation.x -
                      grid_cell_size_ * (static_cast<float>(num_cols_) / 2.0);
    float sambot1_y = sambot1_goal_pose.transform.translation.y -
                      grid_cell_size_ * (static_cast<float>(num_rows_) / 2.0);

    float sambot2_x = sambot2_goal_pose.transform.translation.x -
                      grid_cell_size_ * (static_cast<float>(num_cols_) / 2.0);
    float sambot2_y = sambot2_goal_pose.transform.translation.y -
                      grid_cell_size_ * (static_cast<float>(num_rows_) / 2.0);

    float sambot3_x = sambot3_goal_pose.transform.translation.x -
                      grid_cell_size_ * (static_cast<float>(num_cols_) / 2.0);
    float sambot3_y = sambot3_goal_pose.transform.translation.y -
                      grid_cell_size_ * (static_cast<float>(num_rows_) / 2.0);

    float sambot4_x = sambot4_goal_pose.transform.translation.x -
                      grid_cell_size_ * (static_cast<float>(num_cols_) / 2.0);
    float sambot4_y = sambot4_goal_pose.transform.translation.y -
                      grid_cell_size_ * (static_cast<float>(num_rows_) / 2.0);

    grid_map::Polygon env_polygon;
    env_polygon.setFrameId(spatial_temporal_field_->getFrameId());
    env_polygon.addVertex(grid_map::Position(sambot1_x, sambot1_y));
    env_polygon.addVertex(grid_map::Position(sambot2_x, sambot2_y));
    env_polygon.addVertex(grid_map::Position(sambot3_x, sambot3_y));
    env_polygon.addVertex(grid_map::Position(sambot4_x, sambot4_y));
    env_polygon.addVertex(grid_map::Position(sambot1_x, sambot1_y));

    // Agent
    grid_map::Polygon agent_polygon;
    agent_polygon.setFrameId(agent_field_->getFrameId());
    agent_polygon.addVertex(grid_map::Position(sambot1_x, sambot1_y));
    agent_polygon.addVertex(grid_map::Position(sambot2_x, sambot2_y));
    agent_polygon.addVertex(grid_map::Position(sambot3_x, sambot3_y));
    agent_polygon.addVertex(grid_map::Position(sambot4_x, sambot4_y));
    agent_polygon.addVertex(grid_map::Position(sambot1_x, sambot1_y));

    // Publish polygon messages to visualize
    geometry_msgs::PolygonStamped env_message;
    grid_map::PolygonRosConverter::toMessage(env_polygon, env_message);
    env_polygon_pub_.publish(env_message);

    geometry_msgs::PolygonStamped agent_message;
    grid_map::PolygonRosConverter::toMessage(agent_polygon, agent_message);
    agent_polygon_pub_.publish(agent_message);

    // Publish formation center pose as odometry
    // Calculate formation center in env field frame
    nav_msgs::Odometry formation_center_odom;
    formation_center_odom.header.stamp = ros::Time::now();
    formation_center_odom.header.frame_id = "map";
    formation_center_odom.child_frame_id = "true_formation_center";

    float formation_center_x =
        0.25 *
        (sambot1_goal_pose.transform.translation.x + sambot2_goal_pose.transform.translation.x +
         sambot3_goal_pose.transform.translation.x + sambot4_goal_pose.transform.translation.x);
    float formation_center_y =
        0.25 *
        (sambot1_goal_pose.transform.translation.y + sambot2_goal_pose.transform.translation.y +
         sambot3_goal_pose.transform.translation.y + sambot4_goal_pose.transform.translation.y);

    formation_center_odom.pose.pose.position.x = formation_center_x;
    formation_center_odom.pose.pose.position.y = formation_center_y;
    formation_center_odom.pose.pose.position.z = 0;

    formation_center_odom.pose.pose.orientation.x = 0;
    formation_center_odom.pose.pose.orientation.y = 0;
    formation_center_odom.pose.pose.orientation.z = 0;
    formation_center_odom.pose.pose.orientation.w = 1;

    formation_center_odom_pub_.publish(formation_center_odom);

    // Copy values from environment polygon to agent polugon
    grid_map::PolygonIterator env_iterator(*spatial_temporal_field_, env_polygon);
    grid_map::PolygonIterator agent_iterator(*agent_field_, agent_polygon);

    while (!env_iterator.isPastEnd())
    {
        agent_field_->at("concentration", *agent_iterator) =
            spatial_temporal_field_->at("concentration", *env_iterator);

        ++env_iterator;
        ++agent_iterator;
    }
}

void SpatialTemporalFieldPublisherNode::UpdateFieldState()
{
    // Setup variables used for updates
    Eigen::MatrixXf updated_u = curr_u_;

    Eigen::MatrixXf agent_curr_u_ = (*agent_field_)["concentration"];
    Eigen::MatrixXf updated_agent_u = agent_curr_u_;

    for (int i = 1; i < num_rows_ - 1; i++)
    {
        for (int j = 1; j < num_cols_ - 1; j++)
        {
            updated_u(j, i) =
                curr_u_(j, i) +
                k_ * (dt_ / pow(dx_, 2) *
                          ((curr_u_(j + 1, i) + curr_u_(j - 1, i) + curr_u_(j, i + 1) +
                            curr_u_(j, i - 1) - 4 * curr_u_(j, i))) +
                      vx_ * (dt_ / dx_) * ((curr_u_(j + 1, i) - curr_u_(j, i))) +
                      vy_ * (dt_ / dy_) * (curr_u_(j, i + 1) - curr_u_(j, i)));

            updated_agent_u(j, i) =
                agent_curr_u_(j, i) +
                k_ * (dt_ / pow(dx_, 2) *
                          ((agent_curr_u_(j + 1, i) + agent_curr_u_(j - 1, i) +
                            agent_curr_u_(j, i + 1) + agent_curr_u_(j, i - 1) -
                            4 * agent_curr_u_(j, i))) +
                      vx_ * (dt_ / dx_) * ((agent_curr_u_(j + 1, i) - agent_curr_u_(j, i))) +
                      vy_ * (dt_ / dy_) * (agent_curr_u_(j, i + 1) - agent_curr_u_(j, i)));
        }
    }

    // Update current env field variables
    curr_u_ = updated_u;
    spatial_temporal_field_->add("concentration", curr_u_);

    // Update current agent field variables
    agent_curr_u_ = updated_agent_u;
    agent_field_->add("concentration", agent_curr_u_);
}

void SpatialTemporalFieldPublisherNode::Callback(const std_msgs::BoolConstPtr& trigger_msg)
{
    // If false message received, reset the field to original condition
    if (trigger_msg->data == 0u)
    {
        InitialFieldStateSetup();
    }
    // Update the field state according to the field movement parameters
    else
    {
        CopyViewScope();
        UpdateFieldState();
    }

    // Publish the field after updating
    PublishCurrentField();
}

void SpatialTemporalFieldPublisherNode::PublishCurrentField()
{
    // Environment Field
    // Update timestamp
    spatial_temporal_field_->setTimestamp(ros::Time::now().toNSec());

    // Create message
    grid_map_msgs::GridMap field_msg;
    grid_map::GridMapRosConverter::toMessage(*spatial_temporal_field_, field_msg);
    grid_map_pub_.publish(field_msg);

    // Agent field
    agent_field_->setTimestamp(ros::Time::now().toNSec());

    // Create message
    grid_map_msgs::GridMap agent_field_msg;
    grid_map::GridMapRosConverter::toMessage(*agent_field_, agent_field_msg);
    agent_grid_map_pub_.publish(agent_field_msg);

    // Publish mapping error
    float mapping_error =
        (((*spatial_temporal_field_)["concentration"] - (*agent_field_)["concentration"])
             .cwiseAbs())
            .sum();
    std_msgs::Float32 mapping_error_msg;
    mapping_error_msg.data = mapping_error;
    mapping_error_pub_.publish(mapping_error_msg);
}

}  // namespace sambot_rl

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spatial_temporal_field_publisher");
    ros::NodeHandle pnh("~");

    std::string in_update_trigger_topic = "";
    std::string out_field_topic = "";
    std::string out_agent_field_topic = "";
    std::string initial_field_state_file = "";
    float dx = 0.8;
    float dy = 0.8;
    float vx = -0.6;
    float vy = 0.8;
    float dt = 0.1;
    float k = 1.0;
    int num_rows = 100;
    int num_cols = 100;
    float grid_cell_size = 0.002;

    int bad_params = 0;

    bad_params += !pnh.getParam("in_update_trigger_topic", in_update_trigger_topic);
    bad_params += !pnh.getParam("out_field_topic", out_field_topic);
    bad_params += !pnh.getParam("out_agent_field_topic", out_agent_field_topic);
    bad_params += !pnh.getParam("initial_field_state_file", initial_field_state_file);
    bad_params += !pnh.getParam("dx", dx);
    bad_params += !pnh.getParam("dy", dy);
    bad_params += !pnh.getParam("vx", vx);
    bad_params += !pnh.getParam("vy", vy);
    bad_params += !pnh.getParam("dt", dt);
    bad_params += !pnh.getParam("k", k);
    bad_params += !pnh.getParam("num_rows", num_rows);
    bad_params += !pnh.getParam("num_cols", num_cols);
    bad_params += !pnh.getParam("grid_cell_size", grid_cell_size);

    if (bad_params > 0)
    {
        std::cout << "One or more parameters are not set! Exiting." << std::endl;
        return 1;
    }

    sambot_rl::SpatialTemporalDiffusionParams diff_params{dx, dy, vx, vy, dt, k};

    sambot_rl::SpatialTemporalFieldPublisherNode stfpn(
        in_update_trigger_topic, out_field_topic, out_agent_field_topic, initial_field_state_file,
        diff_params, num_rows, num_cols, grid_cell_size);

    ros::spin();
    return 0;
}
