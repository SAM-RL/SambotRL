#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <sambot_base/MoveTowardGoalAction.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sambot_msgs/NavState.h>

const double PI  =3.141592653589793238463;
const double CELL_TO_METER = 0.0254 * 9;
// const double OFFSET = 12.7279220614;
const double OFFSET = CELL_TO_METER;

const double TOLERANCE_XY = 0.1;
const double TOLERANCE_THETA = 0.2;

const int WAYPOINTS_MAX = 20;
const int WAYPOINTS_LENGTH = 20;
double waypoints[WAYPOINTS_LENGTH][3] = {
  //0-4
  {-4,  4, -PI/2},
  {-4,  3, -PI/4},
  {-3,  3, -PI/4},
  {-2,  2, -PI/4},
  {-2,  1, -PI/2},
  //5-9
  {-2,  0, -PI},
  {-3,  0, -3*PI/4},
  {-4, -1, -PI/2},
  {-4, -2, -PI/2},
  {-4, -3, 0},
  //10-14
  {-3, -3, PI/4},
  {-2, -2, PI/4},
  {-1, -1, PI/4},
  { 0,  0, PI/4},
  { 1, 1,  0},
  //15-19
  { 2, 1,  PI/4},
  { 3, 2,  PI/2},
  { 3, 3,  PI/2},
  { 3, 4,  PI/4},
  { 4, 5,  -PI/2},
};

// const int WAYPOINTS_MAX = 5;
// const int WAYPOINTS_LENGTH = 4;
// double waypoints[WAYPOINTS_LENGTH][3] = {
//   // {1,  0,  0},
//   {-1,  0,  0},
//   { 0,  0, -PI/2},
//   { 0, -1, -PI},
//   {-1,  -1, PI/2},
// };




std::string robot_name[4] = { "sambot1", "sambot2", "sambot3", "sambot4" };
bool active_robots[4] = {true, true, true, true};
// bool active_robots[4] = {false, false, false, true};

double offset[4][2] = {{OFFSET, -OFFSET}, {OFFSET, OFFSET}, {-OFFSET, OFFSET}, {-OFFSET, -OFFSET}};
// double offset[4][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}};

sambot_base::MoveTowardGoalGoal composeGoal(double x, double y, double theta) {
  sambot_base::MoveTowardGoalGoal goal;
  goal.x = x;
  goal.y = y;
  goal.theta = theta;
  // goal.tolerance_xy = TOLERANCE_XY;
  // goal.tolerance_theta = TOLERANCE_THETA;
  return goal;
}

void publishGoalToTF(double x, double y, double theta, std::string name,  tf2_ros::StaticTransformBroadcaster static_broadcaster) {
  tf2::Quaternion q;
  q.setRPY(0.0,0.0,theta);
  q=q.normalize();

  geometry_msgs::TransformStamped transform;

  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = "map";
  transform.child_frame_id = name + "/goal";

  transform.transform.translation.x = x;
  transform.transform.translation.y = y;
  transform.transform.translation.z = 0.0;

  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  static_broadcaster.sendTransform(transform);
}

int main (int argc, char **argv)
{
  ros::init(argc, argv, "move_toward_goal_client");
  ros::NodeHandle nh;

  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  ros::Publisher pub_nav_state = nh.advertise<sambot_msgs::NavState>("/center_goal", 100);

  actionlib::SimpleActionClient<sambot_base::MoveTowardGoalAction> ac1("/sambot1/move_toward_goal_server", true);
  actionlib::SimpleActionClient<sambot_base::MoveTowardGoalAction> ac2("/sambot2/move_toward_goal_server", true);
  actionlib::SimpleActionClient<sambot_base::MoveTowardGoalAction> ac3("/sambot3/move_toward_goal_server", true);
  actionlib::SimpleActionClient<sambot_base::MoveTowardGoalAction> ac4("/sambot4/move_toward_goal_server", true);

  ROS_INFO("Waiting for action server to start.!!!");

  if (active_robots[0]) ac1.waitForServer(); //will wait for infinite time
  if (active_robots[1]) ac2.waitForServer(); //will wait for infinite time
  if (active_robots[2]) ac3.waitForServer(); //will wait for infinite time
  if (active_robots[3]) ac4.waitForServer(); //will wait for infinite time

  ROS_INFO("Action servers are ready, sending goal.");
  ros::Rate r(2.0);
  
  for (int i=0; i<WAYPOINTS_MAX; i++) {
    double pos[4][3] = {0};
    for (int j=0; j<4; j++) {
      for (int k=0; k<2; k++) {
        pos[j][k] = CELL_TO_METER * waypoints[i % WAYPOINTS_LENGTH][k] + offset[j][k];
      }
      pos[j][2] = waypoints[i % WAYPOINTS_LENGTH][2];      
    }

    if (active_robots[0]) ac1.sendGoal(composeGoal(pos[0][0],pos[0][1],pos[0][2]));
    if (active_robots[1]) ac2.sendGoal(composeGoal(pos[1][0],pos[1][1],pos[1][2]));
    if (active_robots[2]) ac3.sendGoal(composeGoal(pos[2][0],pos[2][1],pos[2][2]));
    if (active_robots[3]) ac4.sendGoal(composeGoal(pos[3][0],pos[3][1],pos[3][2]));

    sambot_msgs::NavState msg_nav;
    msg_nav.goal.push_back(CELL_TO_METER * waypoints[i % WAYPOINTS_LENGTH][0]);
    msg_nav.goal.push_back(CELL_TO_METER * waypoints[i % WAYPOINTS_LENGTH][1]);
    pub_nav_state.publish(msg_nav);
    
    for (int i=0; i<4; i++) {
      if (active_robots[i])
        publishGoalToTF(pos[i][0], pos[i][1], pos[i][2], robot_name[i], static_broadcaster);
    }

    bool success = true;
    success &= (active_robots[0]) ? ac1.waitForResult(ros::Duration(30.0)) : true;
    success &= (active_robots[1]) ? ac2.waitForResult(ros::Duration(30.0)) : true;
    success &= (active_robots[2]) ? ac3.waitForResult(ros::Duration(30.0)) : true;
    success &= (active_robots[3]) ? ac4.waitForResult(ros::Duration(30.0)) : true;
    if (!success) break;

    r.sleep();
  }
  
  while (ros::ok()) {
    r.sleep();
    ros::spinOnce();    
  }

  if (!ros::ok()) {
      ac1.cancelGoal();
      ac2.cancelGoal();
      ac3.cancelGoal();
      ac4.cancelGoal();
  }

  //exit
  return 0;
}
