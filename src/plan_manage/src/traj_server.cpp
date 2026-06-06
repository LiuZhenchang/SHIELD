#include "bspline/non_uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "bspline/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int16.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
#include <traj_utils/planning_visualization.h>
#include <geometry_msgs/PoseStamped.h>
// #include <swarmtal_msgs/drone_onboard_command.h>

using shield::NonUniformBspline;
using shield::PlanningVisualization;

ros::Publisher cmd_vis_pub, pos_cmd_pub1, pos_cmd_pub2, traj_pub, swarm_pos_cmd_pub;
nav_msgs::Odometry odom;
quadrotor_msgs::PositionCommand cmd1;
geometry_msgs::PoseStamped cmd2;

// Info of generated traj
vector<NonUniformBspline> traj_;
double traj_duration_;
ros::Time start_time_;
int traj_id_;
int pub_traj_id_;

//Info of flight state
Eigen::Vector3d odom_last_pos_(0.0, 0.0, 0.0);

// Info of replan
bool receive_traj_ = false;
double replan_time_;
bool emergency_stop_ = false;   // LZC added
Eigen::Vector3d position_safe_; // LZC added

// Executed traj, commanded and real ones
vector<Eigen::Vector3d> traj_cmd_, traj_real_;


// Loop correction
Eigen::Matrix3d R_loop;
Eigen::Vector3d T_loop;
bool isLoopCorrection = false;

// Swarm
int drone_id_;
int drone_num_;

// General Setting
std::string frame_id_;

double calcPathLength(const vector<Eigen::Vector3d> &path)
{
  if (path.empty())
    return 0;
  double len = 0.0;
  for (int i = 0; i < path.size() - 1; ++i)
  {
    len += (path[i + 1] - path[i]).norm();
  }
  return len;
}

void displayTrajWithColor(
    vector<Eigen::Vector3d> path, double resolution, Eigen::Vector4d color, int id)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE_LIST;
  mk.action = visualization_msgs::Marker::DELETE;
  mk.id = drone_id_ * 10 + id;
  traj_pub.publish(mk);

  mk.action = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;
  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;
  geometry_msgs::Point pt;
  for (int i = 0; i < int(path.size()); i++)
  {
    pt.x = path[i](0);
    pt.y = path[i](1);
    pt.z = path[i](2);
    mk.points.push_back(pt);
  }
  traj_pub.publish(mk);
  ros::Duration(0.001).sleep();
}

void drawFOV(const vector<Eigen::Vector3d> &list1, const vector<Eigen::Vector3d> &list2)
{
  visualization_msgs::Marker mk;
  mk.header.frame_id = frame_id_;
  mk.header.stamp = ros::Time::now();
  mk.id = 0;
  mk.ns = "current_pose";
  mk.type = visualization_msgs::Marker::LINE_LIST;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  // auto color = PlanningVisualization::getColor((drone_id_ - 1) / double(drone_num_));
  Eigen::Vector4d color(0, 0, 0, 1);

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = 0.04;
  mk.scale.y = 0.04;
  mk.scale.z = 0.04;

  // Clean old marker
  mk.action = visualization_msgs::Marker::DELETE;
  cmd_vis_pub.publish(mk);

  if (list1.size() == 0)
    return;

  // Pub new marker
  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i)
  {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  mk.action = visualization_msgs::Marker::ADD;
  cmd_vis_pub.publish(mk);
}

void drawCmd(const Eigen::Vector3d &pos, const Eigen::Vector3d &vec, const int &id,
             const Eigen::Vector4d &color)
{
  visualization_msgs::Marker mk_state;
  mk_state.header.frame_id = frame_id_;
  mk_state.header.stamp = ros::Time::now();
  mk_state.id = id;
  mk_state.type = visualization_msgs::Marker::ARROW;
  mk_state.action = visualization_msgs::Marker::ADD;

  mk_state.pose.orientation.w = 1.0;
  mk_state.scale.x = 0.1;
  mk_state.scale.y = 0.2;
  mk_state.scale.z = 0.5;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk_state.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk_state.points.push_back(pt);

  mk_state.color.r = color(0);
  mk_state.color.g = color(1);
  mk_state.color.b = color(2);
  mk_state.color.a = color(3);

  cmd_vis_pub.publish(mk_state);
}

void replanCallback(std_msgs::Int16 msg)
{
  // Informed of new replan, end the current traj after some time
  const double time_out = 0.2;
  ros::Time time_now = ros::Time::now();
  double t_stop = (time_now - start_time_).toSec() + time_out + replan_time_;
  traj_duration_ = min(t_stop, traj_duration_);
  if (msg.data == 1) // LZC Added
    emergency_stop_ = true;
}

void odomCallbck(const nav_msgs::Odometry &msg)
{
  odom = msg;
  /************************** LZC Added *************************/
  if (!emergency_stop_)
  {
    position_safe_.x() = odom.pose.pose.position.x;
    position_safe_.y() = odom.pose.pose.position.y;
    position_safe_.z() = odom.pose.pose.position.z;
  }
  /**************************************************************/
  Eigen::Vector3d odom_pos(odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
  traj_real_.push_back(odom_pos);
  odom_last_pos_ = odom_pos;

  // if (traj_real_.size() > 10000)
  //   traj_real_.erase(traj_real_.begin(), traj_real_.begin() + 1000);
}


void visCallback(const ros::TimerEvent &e)
{
  // Draw the executed traj (desired state)
  displayTrajWithColor(traj_cmd_, 0.1, Eigen::Vector4d(0, 0, 1, 1), pub_traj_id_);
  displayTrajWithColor(traj_real_, 0.5, Eigen::Vector4d(1, 1, 1, 0.8), 100 + pub_traj_id_);
}

void bsplineCallback(const bspline::BsplineConstPtr &msg)
{
  // Received traj should have ascending traj_id
  if (msg->traj_id <= traj_id_)
  {
    ROS_ERROR("out of order bspline.");
    return;
  }
  emergency_stop_ = false; // LZC added

  // Parse the msg
  Eigen::MatrixXd pos_pts(msg->pos_pts.size(), 3);
  Eigen::VectorXd knots(msg->knots.size());
  for (int i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }
  for (int i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(i, 0) = msg->pos_pts[i].x;
    pos_pts(i, 1) = msg->pos_pts[i].y;
    pos_pts(i, 2) = msg->pos_pts[i].z;
  }
  NonUniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  for (int i = 0; i < msg->yaw_pts.size(); ++i)
    yaw_pts(i, 0) = msg->yaw_pts[i];
  NonUniformBspline yaw_traj(yaw_pts, 3, msg->yaw_dt);
  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  traj_.clear();
  traj_.push_back(pos_traj);
  traj_.push_back(traj_[0].getDerivative());
  traj_.push_back(traj_[1].getDerivative());
  traj_.push_back(yaw_traj);
  traj_.push_back(yaw_traj.getDerivative());
  traj_.push_back(traj_[2].getDerivative());
  traj_duration_ = traj_[0].getTimeSum();

  receive_traj_ = true;


}

void cmdCallback(const ros::TimerEvent &e)
{
  // No publishing before receive traj data
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec();
  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), jer(Eigen::Vector3d::Zero());
  double yaw = 0.0, yawdot = 0.0;

  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    // Current time within range of planned traj
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);
    yaw = traj_[3].evaluateDeBoorT(t_cur)[0];
    yawdot = traj_[4].evaluateDeBoorT(t_cur)[0];
    jer = traj_[5].evaluateDeBoorT(t_cur);
  }
  else if (t_cur >= traj_duration_)
  {
    // Current time exceed range of planned traj
    // keep publishing the final position and yaw
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();
    yaw = traj_[3].evaluateDeBoorT(traj_duration_)[0];
    yawdot = 0.0;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }


  if (isLoopCorrection)
  {
    pos = R_loop.transpose() * (pos - T_loop);
    vel = R_loop.transpose() * vel;
    acc = R_loop.transpose() * acc;

    Eigen::Vector3d yaw_dir(cos(yaw), sin(yaw), 0);
    yaw_dir = R_loop.transpose() * yaw_dir;
    yaw = atan2(yaw_dir[1], yaw_dir[0]);
  }
  // publish command for Marsim
  cmd1.header.stamp = time_now;
  cmd1.trajectory_id = traj_id_;
  cmd1.position.x = pos(0);
  cmd1.position.y = pos(1);
  cmd1.position.z = pos(2);
  cmd1.velocity.x = vel(0);
  cmd1.velocity.y = vel(1);
  cmd1.velocity.z = vel(2);
  cmd1.acceleration.x = acc(0);
  cmd1.acceleration.y = acc(1);
  cmd1.acceleration.z = acc(2);
  cmd1.yaw = yaw;
  cmd1.yaw_dot = yawdot;
  pos_cmd_pub1.publish(cmd1);

  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(0,Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw,Eigen::Vector3d::UnitZ())); 
  Eigen::Quaterniond q=yawAngle*pitchAngle*rollAngle;

  // publish command for gazebo
  cmd2.header.stamp= time_now;
  cmd2.pose.position.x = pos(0);
  cmd2.pose.position.y = pos(1);
  cmd2.pose.position.z = pos(2);
  cmd2.pose.orientation.w = q.w();
  cmd2.pose.orientation.x = q.x();
  cmd2.pose.orientation.y = q.y();
  cmd2.pose.orientation.z = q.z();
  pos_cmd_pub2.publish(cmd2);

  // Draw cmd
  Eigen::Vector3d dir(cos(yaw), sin(yaw), 0.0);
  drawCmd(pos, dir, 2, Eigen::Vector4d(1, 1, 0, 0.7));
  // drawCmd(pos, vel, 0, Eigen::Vector4d(0, 1, 0, 1));
  // drawCmd(pos, acc, 1, Eigen::Vector4d(0, 0, 1, 1));
  // drawCmd(pos, pos_err, 3, Eigen::Vector4d(1, 1, 0, 0.7));

  // Record info of the executed traj
  if (traj_cmd_.size() == 0)
  {
    // Add the first position
    traj_cmd_.push_back(pos);
  }
  else if ((pos - traj_cmd_.back()).norm() > 1e-6)
  {
    // Add new different commanded position
    traj_cmd_.push_back(pos);

  }


  // if (traj_cmd_.size() > 100000)
  //   traj_cmd_.erase(traj_cmd_.begin(), traj_cmd_.begin() + 1000);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);
  ros::Subscriber replan_sub = node.subscribe("planning/replan", 10, replanCallback);
  ros::Subscriber odom_sub = node.subscribe("/odom_world", 50, odomCallbck);

  cmd_vis_pub = node.advertise<visualization_msgs::Marker>("planning/position_cmd_vis", 10);
  pos_cmd_pub1 = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd1", 10);
  pos_cmd_pub2 = node.advertise<geometry_msgs::PoseStamped>("/position_cmd2", 10);
  traj_pub = node.advertise<visualization_msgs::Marker>("planning/travel_traj", 10);

  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);
  ros::Timer vis_timer = node.createTimer(ros::Duration(0.25), visCallback);

  nh.param("traj_server/pub_traj_id", pub_traj_id_, -1);
  nh.param("traj_server/drone_id", drone_id_, 1);
  nh.param("traj_server/drone_num", drone_num_, 1);
  nh.param("fsm/replan_time", replan_time_, 0.1);
  nh.param("loop_correction/isLoopCorrection", isLoopCorrection, false);
  nh.param("general/frame_id", frame_id_, string("map"));


  ROS_WARN("[Traj server]: init...");
  ros::Duration(1.0).sleep();


  R_loop = Eigen::Quaterniond(1, 0, 0, 0).toRotationMatrix();
  T_loop = Eigen::Vector3d(0, 0, 0);

  ROS_WARN("[Traj server]: ready.");
  ros::spin();

  return 0;
}
