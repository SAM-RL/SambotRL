#pragma once

#include "cnpy.h"
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <Eigen/Dense>

namespace sambot_rl
{

struct SpatialTemporalDiffusionParams
{
    // TODO (deepak): Add comments explaining each of these parameters
    float dx;
    float dy;
    float vx;
    float vy;
    float dt;
    float k;
};

/**
 * Node for publishing the Spatial-Temporal field to be used in Reinforcement Learning tasks as a
 * GridMap. This node reads the initial state of the field from the provided .npy file, augments it
 * with a second source of concentration and updates it at each time step according to the provided
 * diffusion parameters.
 */
class SpatialTemporalFieldPublisherNode
{
public:
    /**
     * Constructor
     *
     * @param in_update_trigger_topic (const std::string&): Topic to subscribe to for field update
     * triggers
     * @param out_field_topic (const std::string&): Output topic on which to publish the
     * spatial-temporal field's GridMap
     * @param initial_field_state_file (const std::string&): Path to .npy file that contains the
     * initial field concentration information
     * @param diff_params (const SpatialTemporalDiffusionParams&): Diffusion parameters used for
     * updating the field at each timestep.
     * @param num_rows (int): Number of rows in the spatial-temporal field
     * @param num_cols (int): Number of columns in the spatial-temporal field
     * @param grid_cell_size (float): Length of the side (in meters) of one cell inside the GridMap.
     */
    SpatialTemporalFieldPublisherNode(
        const std::string& in_update_trigger_topic, const std::string& out_field_topic,
        const std::string& out_agent_field_topic, const std::string& initial_field_state_file,
        const SpatialTemporalDiffusionParams& diff_params, int num_rows, int num_cols,
        float grid_cell_size);

    /**
     * Destructor
     */
    ~SpatialTemporalFieldPublisherNode();

    /**
     * Callback function for triggering field update
     *
     * @param update_trigger_msg (const std_msgs::BoolConstPtr&): When true, a field update is
     * triggered. When false, field is reset to initial conditions.
     */
    void Callback(const std_msgs::BoolConstPtr& update_trigger_msg);

    void CopyViewScope();

private:
    // Grid Map params
    int num_rows_ = 100;
    int num_cols_ = 100;
    float grid_cell_size_ = 0.05;  // in meters
    std::string initial_field_state_file_ = "";

    // Diffusion parameters
    float dx_ = 0.8;
    float dy_ = 0.8;
    float vx_ = -0.6;
    float vy_ = 0.8;
    float dt_ = 0.1;
    float k_ = 1.0;

    // Object state
    Eigen::MatrixXf curr_u_;
    Eigen::MatrixXf agent_curr_u_;

    // Grid Map objects
    std::shared_ptr<grid_map::GridMap> spatial_temporal_field_;
    std::shared_ptr<grid_map::GridMap> agent_field_;

    // Subscriber for trigger
    ros::Subscriber trigger_sub_;

    // Grid Map Publisher
    ros::Publisher grid_map_pub_;
    ros::Publisher agent_grid_map_pub_;
    ros::Publisher env_polygon_pub_;
    ros::Publisher agent_polygon_pub_;
    ros::Publisher mapping_error_pub_;
    ros::Publisher formation_center_odom_pub_;

    // TF2 objects
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    /**
     * Function that sets up the initial state of the field. This function reads the values from the
     * .npy file at `initial_field_state_file_` and augments it with a second concentration source.
     * This function is called once from inside the constructor.
     */
    void InitialFieldStateSetup();

    /**
     * Publishes the current state of the spatial-temporal field as a GridMap
     */
    void PublishCurrentField();

    /**
     * Updates the spatial-temporal field given the diffusion parameters
     */
    void UpdateFieldState();

};  // class SpatialTemporalFieldPublisherNode

}  // namespace sambot_rl
