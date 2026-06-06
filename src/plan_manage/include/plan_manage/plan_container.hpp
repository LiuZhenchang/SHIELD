#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <bspline/non_uniform_bspline.h>
#include <poly_traj/polynomial_traj.h>
#include <active_perception/traj_visibility.h>

using std::vector;

namespace shield {
class GlobalTrajData {
private:
public:
  vector<NonUniformBspline> local_traj_;

  double global_duration_;
  ros::Time global_start_time_;
  double local_start_time_, local_end_time_;
  double time_change_;
  double last_time_inc_;

  GlobalTrajData(/* args */) {
  }

  ~GlobalTrajData() {
  }

  bool localTrajReachTarget() {
    return fabs(local_end_time_ - global_duration_) < 1e-3;
  }


  void setLocalTraj(const NonUniformBspline& traj, const double& local_ts, const double& local_te,
      const double& time_change) {
    local_traj_.resize(3);
    local_traj_[0] = traj;
    local_traj_[1] = local_traj_[0].getDerivative();
    local_traj_[2] = local_traj_[1].getDerivative();

    local_start_time_ = local_ts;
    local_end_time_ = local_te;
    global_duration_ += time_change;
    time_change_ += time_change;
    last_time_inc_ = time_change;
  }
};

struct PlanParameters {
  /* planning algorithm parameters */
  double max_vel_, max_acc_, max_jerk_;  // physical limits
  double accept_vel_, accept_acc_;

  double max_yawdot_;
  double ctrl_pt_dist;     // distance between adjacient B-spline control points
  int bspline_degree_;
  bool min_time_;

  double clearance_;
  int dynamic_;
  /* processing time */
  double time_search_ = 0.0;
  double time_optimize_ = 0.0;
  double time_adjust_ = 0.0;

  double relax_time1_, relax_time2_;
};

struct LocalTrajData {
  /* info of generated traj */

  int traj_id_;
  double duration_;
  ros::Time start_time_;
  Eigen::Vector3d start_pos_;
  NonUniformBspline position_traj_, velocity_traj_, acceleration_traj_, yaw_traj_, yawdot_traj_, yawdotdot_traj_, tmp_traj_;
};

// structure of trajectory info
struct LocalTrajState {
  Eigen::Vector3d pos, vel, acc;
  double yaw, yawdot;
  int id;
};

class LocalTrajServer {
private:
  LocalTrajData traj1_, traj2_;

public:
  LocalTrajServer(/* args */) {
    traj1_.traj_id_ = 0;
    traj2_.traj_id_ = 0;
  }
  ~LocalTrajServer() {
  }

  void addTraj(const LocalTrajData& traj) {
    if (traj1_.traj_id_ == 0) {
      // receive the first traj, save in traj1
      traj1_ = traj;
    } else {
      traj2_ = traj;
    }
  }

  bool evaluate(const ros::Time& time, LocalTrajState& traj_state) {
    if (traj1_.traj_id_ == 0) {
      // not receive traj yet
      return false;
    }

    if (traj2_.traj_id_ != 0 && time > traj2_.start_time_) {
      // have traj2 AND time within range of traj2. should use traj2 now
      traj1_ = traj2_;
      traj2_.traj_id_ = 0;
    }

    double t_cur = (time - traj1_.start_time_).toSec();
    if (t_cur < 0) {
      cout << "[Traj server]: invalid time." << endl;
      return false;
    } else if (t_cur < traj1_.duration_) {
      // time within range of traj 1
      traj_state.pos = traj1_.position_traj_.evaluateDeBoorT(t_cur);
      traj_state.vel = traj1_.velocity_traj_.evaluateDeBoorT(t_cur);
      traj_state.acc = traj1_.acceleration_traj_.evaluateDeBoorT(t_cur);
      traj_state.yaw = traj1_.yaw_traj_.evaluateDeBoorT(t_cur)[0];
      traj_state.yawdot = traj1_.yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      traj_state.id = traj1_.traj_id_;
      return true;
    } else {
      traj_state.pos = traj1_.position_traj_.evaluateDeBoorT(traj1_.duration_);
      traj_state.vel.setZero();
      traj_state.acc.setZero();
      traj_state.yaw = traj1_.yaw_traj_.evaluateDeBoorT(traj1_.duration_)[0];
      traj_state.yawdot = 0;
      traj_state.id = traj1_.traj_id_;
      return true;
    }
  }

  void resetDuration() {
    ros::Time now = ros::Time::now();
    if (traj1_.traj_id_ != 0) {
      double t_stop = (now - traj1_.start_time_).toSec();
      traj1_.duration_ = min(t_stop, traj1_.duration_);
    }
    if (traj2_.traj_id_ != 0) {
      double t_stop = (now - traj2_.start_time_).toSec();
      traj2_.duration_ = min(t_stop, traj2_.duration_);
    }
  }
};

class MidPlanData {
public:
  MidPlanData(/* args */) {
  }
  ~MidPlanData() {
  }

  vector<Eigen::Vector3d> global_waypoints_;

  // initial trajectory segment
  NonUniformBspline initial_local_segment_;
  vector<Eigen::Vector3d> local_start_end_derivative_;

  // middle plan path
  vector<Eigen::Vector3d> kino_path_;
  vector<Eigen::Vector3d> poly_path_;

  // visibility constraint
  vector<Eigen::Vector3d> block_pts_;
  Eigen::MatrixXd ctrl_pts_;
  NonUniformBspline no_visib_traj_;
  vector<VisiblePair> visib_pairs_;
  ViewConstraint view_cons_;

  // heading planning
  vector<double> path_yaw_;
  double dt_yaw_;
  double dt_yaw_path_;
};

class SwarmData {
public:
  SwarmData(/* args */) {
  }
  ~SwarmData() {
  }

  void init(int id, int num) {
    drone_id_ = id;
    drone_num_ = num;
    swarm_trajs_.resize(drone_num_);
    receive_flags_ = vector<bool>(drone_num_, false);
  }

  void getValidTrajs(vector<NonUniformBspline>& trajs) {
    // Retrieve only valid trajs
    trajs.clear();
    for (size_t i = 0; i < drone_num_; ++i) {
      if (receive_flags_[i] == true) {
        trajs.push_back(swarm_trajs_[i]);
      }
    }
  }

  void resetReceiveFlag() {
    fill(receive_flags_.begin(), receive_flags_.end(), false);
  }

  int drone_id_;
  int drone_num_;
  vector<NonUniformBspline> swarm_trajs_;
  vector<bool> receive_flags_;
};

}  // namespace shield

#endif