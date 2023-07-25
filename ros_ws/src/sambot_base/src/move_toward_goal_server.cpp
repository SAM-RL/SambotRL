#include <math.h>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/server/simple_action_server.h>
#include <sambot_base/MoveTowardGoalAction.h>
#include <sensor_msgs/JointState.h>
#include <sambot_msgs/NavState.h>
#include "sambot_base/UpdatePID.h"
#include <std_msgs/Float32.h>

#define YAW_RATE_WINDOW_SIZE 3UL
#define WHEEL_VELOCITY_WINDOW_SIZE 3UL
#define MAX_VEL_FULL_STOP 0.05

#define FORWARD_SPEED 0.6
#define ROTATING_SPEED 0.75
#define GAIN 1.0
#define TRIM 0.0
#define STOP_RATE 2.5

#define MOTOR_SPEED_STEP 0.02

#define TOLERANCE_YAW_INPLACE 0.3
#define TOLERANCE_YAW_MOVING 0.5
#define TOLERANCE_XY 0.15

#define OPERATION_ZONE_X 2
#define OPERATION_ZONE_Y 2

#define IGNORE_DESTINATION_ORIENTATION true

class MoveTowardGoalAction
{
  public:
    
    enum RobotState { IDLE=0, DRIVING_STRAIGHT=1, ROTATING_INPLACE=2 };
    const std::string state_names[3] = {"IDLE", "DRIVING_STRAIGHT", "ROTATING_INPLACE"};
    
    struct Pose2D {
      double x;
      double y;
      double yaw;
    };

    MoveTowardGoalAction(std::string name, std::string ns, tf2_ros::Buffer* buffer_ptr) : 
      as_(nh_, name, boost::bind(&MoveTowardGoalAction::executeCB, this, _1), false),
      server_name_(name), server_namespace_(ns), tf_buffer_ptr_(buffer_ptr)
    {      
      nh_.param<double>(server_namespace_ +"/tolerance_yaw_inplace", tolerance_yaw_inplace_, TOLERANCE_YAW_INPLACE);
      nh_.param<double>(server_namespace_ +"/tolerance_yaw_moving", tolerance_yaw_moving_, TOLERANCE_YAW_MOVING);
      nh_.param<double>(server_namespace_ +"/tolerance_xy", tolerance_xy_, TOLERANCE_XY);
      nh_.param<double>(server_namespace_ +"/stop_rate", stop_rate, STOP_RATE);
      nh_.param<double>(server_namespace_ +"/gain", gain_, GAIN);
      nh_.param<double>(server_namespace_ +"/trim", trim_, TRIM);
      nh_.param<double>(server_namespace_ +"/forward_speed", forward_speed, FORWARD_SPEED);
      nh_.param<double>(server_namespace_ +"/rotating_speed", rotating_speed, ROTATING_SPEED);

      pub_motor_left = nh_.advertise<std_msgs::Float32>("motor_left", 10);
      pub_motor_right = nh_.advertise<std_msgs::Float32>("motor_right", 10);
      pub_nav_state = nh_.advertise<sambot_msgs::NavState>(server_namespace_ + "/navigation", 100);
      sub_measured_joint_states_ = nh_.subscribe("measured_joint_states", 10, &MoveTowardGoalAction::measuredJointStatesCallback, this);
        
      as_.start();
      ROS_INFO("%s: action server has started ...", server_name_.c_str());
    }

    ~MoveTowardGoalAction(void) {}

    void measuredJointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg_joint_states)
    {
        double wheel_vel =  std::max(abs(msg_joint_states->velocity[0]), abs(msg_joint_states->velocity[1]));
        getAverageWheelVelocity(&wheel_vel);
    }

    void executeCB(const sambot_base::MoveTowardGoalGoalConstPtr &goal)
    {
      ros::Rate r(20.0);
      // Obtain & validate goal pose
      goal_pose_ = getGoalPose(goal);
      if (!isValidPose(goal_pose_)) {
        as_.setAborted(action_result_, "Invalid Goal: target pose is not within robot operation zone");
        return;
      }

      // Make sure robot pose is available before proceeding
      while (current_pose_timestamp_ == 0 ) {
        ROS_INFO("%s: Waiting for current pose", server_name_.c_str());
        r.sleep();
      }

      // Set initial robot state as IDLE
      setRobotState(IDLE);
      updateRobotMotion();

      // Start action loop
      ROS_INFO("%s: New goal received. Moving to target pose ....", server_name_.c_str());
      destination_reached_ = false;
    
      while (ros::ok()) {
        // -- Print State Statistic
        updateTargetYaw();
        printActionStatistics();
        switch (state_) {
          case DRIVING_STRAIGHT: {
            // continue to drive forward unless reaching destination or heading is out of tolerance
            if (desiredPositionReached() || !desiredHeadingReached(false)) {
              setRobotState(IDLE);
            }
          }          
          break;
          case ROTATING_INPLACE: {
            // continue to rotate until yaw rate is slow enough and heading is approaching
            if (desiredHeadingReached(true)) {
              setRobotState(IDLE);
            }
          }          
          break;
          case IDLE: {
            // wait for the robot to fully stop before transitioning to other state
            if (isFullStop()) {
              if (desiredHeadingReached(true)) {
                if (desiredPositionReached()) {
                  destination_reached_ = true;
                } else {
                    setRobotState(DRIVING_STRAIGHT);
                }
              } else {     
                  setRobotState(ROTATING_INPLACE);                
              }
            }
          }          
          break;          
          default:
          break;
        }
        // -- Check if action completed, break out of control loop
        if (destination_reached_) break;
        updateRobotMotion();
        r.sleep();
      }

      // Report result to client
      if(destination_reached_)
      {
        ROS_INFO("%s: Action Succeeded", server_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(action_result_);
      } else {
        ROS_INFO("%s: Action Aborted", server_name_.c_str());
        as_.setAborted(action_result_);
      }

      return;
    }

    bool isFullStop() {
      return getAverageWheelVelocity() <= MAX_VEL_FULL_STOP;
    }

    void updateRobotMotion() {
      std_msgs::Float32 motor_cmd_left, motor_cmd_right;
      switch (state_)
      {
        case DRIVING_STRAIGHT: {
            motor_cmd_left.data = gain_ * (forward_speed + trim_);
            motor_cmd_right.data = gain_ * (forward_speed - trim_);
            pub_motor_left.publish(motor_cmd_left); pub_motor_right.publish(motor_cmd_right);
          }
          break;
        case ROTATING_INPLACE: {
            ros::Rate r(stop_rate);          
            if (getYawDifference(target_yaw_,current_pose_.yaw) > 0) {
              motor_cmd_left.data = -gain_ * (rotating_speed + trim_); motor_cmd_right.data = gain_ * (rotating_speed + trim_);
            } else {
              motor_cmd_left.data = gain_ * (rotating_speed + trim_); motor_cmd_right.data = -gain_ * (rotating_speed + trim_);
            }
            pub_motor_left.publish(motor_cmd_left); pub_motor_right.publish(motor_cmd_right);   
            r.sleep();
            motor_cmd_left.data = 0; motor_cmd_right.data = 0;
            pub_motor_left.publish(motor_cmd_left); pub_motor_right.publish(motor_cmd_right);   
            r.sleep();
          }
          break;      
        default: {
            motor_cmd_left.data = 0;
            motor_cmd_right.data = 0;
            pub_motor_left.publish(motor_cmd_left); pub_motor_right.publish(motor_cmd_right);
          }
          break;
      }
    }

    void setRobotState(RobotState new_state) {
      state_ = new_state;
    }

    bool desiredPositionReached() {
      return abs(current_pose_.x-goal_pose_.x) <= tolerance_xy_ && abs(current_pose_.y-goal_pose_.y) <= tolerance_xy_;
    }

    bool desiredHeadingReached(bool inplace = true) {
      if (IGNORE_DESTINATION_ORIENTATION && desiredPositionReached()) return true;
      return abs(getYawDifference(current_pose_.yaw, target_yaw_)) <= (inplace ? tolerance_yaw_inplace_ : tolerance_yaw_moving_);
    }

    bool isValidPose(Pose2D pose) {
      return abs(pose.x) <= OPERATION_ZONE_X && abs(pose.y) <= OPERATION_ZONE_Y;
    }

    void updateTargetYaw() {
      target_yaw_ = desiredPositionReached() ? goal_pose_.yaw : atan2(goal_pose_.y-current_pose_.y,goal_pose_.x-current_pose_.x);
    }

    Pose2D getGoalPose(const sambot_base::MoveTowardGoalGoalConstPtr &goal) {
      return {goal->x, goal->y, goal->theta};
    }

    void printActionStatistics() {
      ROS_INFO("%s: dx:%.4f, dy:%.4f, dyaw:%.4f | yaw_rate:%.4f (rad/sec), velocity:%.4f [%f]",
        getStateName(state_).c_str(),
        goal_pose_.x - current_pose_.x,
        goal_pose_.y - current_pose_.y,
        getYawDifference(target_yaw_,current_pose_.yaw),
        getAverageYawRate(),
        getAverageWheelVelocity(),
        current_pose_timestamp_
      );      
    }

    std::string getStateName(RobotState state) {
      if (state == IDLE) return "IDLE";
      else if (state == DRIVING_STRAIGHT) return "DRIVING_STRAIGHT";
      else if (state == ROTATING_INPLACE) return "ROTATING_INPLACE";
      else return "UNKNOWN";
    }

    double getYawDifference(double angle1, double angle2) {
      return atan2(sin(angle1-angle2), cos(angle1-angle2));
    }

    double getAverageYawRate(double* sample_ptr = nullptr) {
      if (sample_ptr != nullptr) {
        yaw_rate_total_ += *sample_ptr;
        if (yaw_rate_num_samples_ < YAW_RATE_WINDOW_SIZE)
            yaw_rate_samples_[yaw_rate_num_samples_++] = *sample_ptr;
        else
        {
            double& oldest = yaw_rate_samples_[yaw_rate_num_samples_++ % YAW_RATE_WINDOW_SIZE];
            yaw_rate_total_ -= oldest;
            oldest = *sample_ptr;
        }      
      }
      return yaw_rate_total_ / std::min(yaw_rate_num_samples_, YAW_RATE_WINDOW_SIZE);
    }

    double getAverageWheelVelocity(double* sample_ptr = nullptr) {
      if (sample_ptr != nullptr) {
        vel_total_ += *sample_ptr;
        if (vel_num_samples_ < WHEEL_VELOCITY_WINDOW_SIZE)
            vel_samples_[vel_num_samples_++] = *sample_ptr;
        else
        {
            double& oldest = vel_samples_[vel_num_samples_++ % WHEEL_VELOCITY_WINDOW_SIZE];
            vel_total_ -= oldest;
            oldest = *sample_ptr;
        }      
      }
      return vel_total_ / std::min(vel_num_samples_, WHEEL_VELOCITY_WINDOW_SIZE);
    }

    void execute() {
      // update pose
      if (updateCurrentPose() && previous_pose_timestamp_ != 0 && abs(current_pose_timestamp_-previous_pose_timestamp_)>0.001) {
          double yaw_rate = getYawDifference(current_pose_.yaw, previous_pose_.yaw) / (current_pose_timestamp_-previous_pose_timestamp_);        
          getAverageYawRate(&yaw_rate);
      }
      // send navigation status
      sambot_msgs::NavState msg_nav;
      msg_nav.goal.push_back(goal_pose_.x);
      msg_nav.goal.push_back(goal_pose_.y);
      msg_nav.position.push_back(current_pose_.x);
      msg_nav.position.push_back(current_pose_.y);
      msg_nav.idle = (state_ == IDLE) ? true : false;
      msg_nav.status = getStateName(state_);
      pub_nav_state.publish(msg_nav);
    }

    bool updateCurrentPose() {
      try {        
          // lookup transform
          geometry_msgs::TransformStamped transform = tf_buffer_ptr_->lookupTransform("map", server_namespace_.substr(1) + "/chassis", ros::Time());
          tf2::Quaternion q(
              transform.transform.rotation.x,
              transform.transform.rotation.y,
              transform.transform.rotation.z,
              transform.transform.rotation.w
          );
          
          // calculate yaw from quaternion
          tf2::Matrix3x3 m(q);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);
          
          // update pose
          previous_pose_ = current_pose_;
          previous_pose_timestamp_ = current_pose_timestamp_;

          current_pose_.x = transform.transform.translation.x;
          current_pose_.y = transform.transform.translation.y;
          current_pose_.yaw = yaw;

          current_pose_timestamp_ = transform.header.stamp.toSec();

          // ROS_INFO("(%.3f, %.3f, %.3f)", current_pose_.x, current_pose_.y, current_pose_.yaw);
          // tf_received = true;
          return true;
      }
      catch (tf2::TransformException &ex) {
        // ROS_WARN("%s",ex.what());
        return false;
      }  
    }
  
  private:

    // ROS setup
    ros::NodeHandle nh_;
    std::string server_name_;
    std::string server_namespace_;

    ros::Subscriber sub_measured_joint_states_;
    ros::Publisher pub_motor_left, pub_motor_right, pub_nav_state;
    
    // Action Server
    actionlib::SimpleActionServer<sambot_base::MoveTowardGoalAction> as_;
    sambot_base::MoveTowardGoalFeedback action_feedback_;
    sambot_base::MoveTowardGoalResult action_result_;
    sambot_base::MoveTowardGoalGoal action_goal_;

    // TF Transform Listener
    tf2_ros::Buffer* tf_buffer_ptr_;
    double current_pose_timestamp_{0};
    double previous_pose_timestamp_{0};
 
    // Robot Status
    RobotState state_ {IDLE};
    Pose2D current_pose_;
    Pose2D previous_pose_;
    Pose2D goal_pose_;
    bool destination_reached_ {false};
    double target_yaw_ {0};

    // Sampling Yaw Rates with Moving Window Average;
    double yaw_rate_samples_[YAW_RATE_WINDOW_SIZE];
    unsigned long yaw_rate_num_samples_{0};
    double yaw_rate_total_{0};

    // Sampling Wheel Angular Velocities with Moving Window Average;
    double vel_samples_[WHEEL_VELOCITY_WINDOW_SIZE];
    unsigned long vel_num_samples_{0};
    double vel_total_{0};

    // Parameters - In-Place Rotation
    double tolerance_yaw_inplace_;
    double tolerance_yaw_moving_;
    double tolerance_xy_; 
    
    double gain_, trim_;
    double forward_speed;
    double rotating_speed;
    double stop_rate;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, ros::this_node::getNamespace() + "move_toward_goal_server");
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  MoveTowardGoalAction move_toward_goal(ros::this_node::getName(), ros::this_node::getNamespace(), &tfBuffer);
  
  ros::Rate rate(10);
  
  while(ros::ok()){
    move_toward_goal.execute();
    rate.sleep();
    ros::spinOnce();      //Notice this
  } 

  return 0;
}