// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>

#include <thread>
#include <visualization_msgs/Marker.h>

namespace shield
{
  // SECTION interfaces for setup and query

  FastPlannerManager::FastPlannerManager()
  {
  }

  FastPlannerManager::~FastPlannerManager()
  {
    std::cout << "des manager" << std::endl;
  }

  void FastPlannerManager::initPlanModules(ros::NodeHandle &nh)
  {
    /* read algorithm parameters */
    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/accept_vel", pp_.accept_vel_, pp_.max_vel_ + 0.5);
    nh.param("manager/accept_acc", pp_.accept_acc_, pp_.max_acc_ + 0.5);
    nh.param("manager/max_yawdot", pp_.max_yawdot_, -1.0);
    nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
    nh.param("manager/clearance_threshold", pp_.clearance_, -1.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/bspline_degree", pp_.bspline_degree_, 3);
    nh.param("manager/min_time", pp_.min_time_, false);
    nh.param("manager/relax_time1", pp_.relax_time1_, 0.5);
    nh.param("manager/relax_time2", pp_.relax_time2_, 0.5);

    bool use_geometric_path, use_kinodynamic_path, use_optimization;
    nh.param("manager/use_geometric_path", use_geometric_path, false);
    nh.param("manager/use_kinodynamic_path", use_kinodynamic_path, false);
    nh.param("manager/use_optimization", use_optimization, false);

    local_data_.traj_id_ = 0;
    sdf_map_.reset(new SDFMap);
    sdf_map_->initMap(nh);
    edt_environment_.reset(new EDTEnvironment);
    edt_environment_->setMap(sdf_map_);

    if (use_geometric_path)
    {
      path_finder_.reset(new Astar);
      // path_finder_->setParam(nh);
      // path_finder_->setEnvironment(edt_environment_);
      // path_finder_->init();
      path_finder_->init(nh, edt_environment_);
    }

    if (use_kinodynamic_path)
    {
      kino_path_finder_.reset(new KinodynamicAstar);
      kino_path_finder_->setParam(nh);
      kino_path_finder_->setEnvironment(edt_environment_);
      kino_path_finder_->init();
    }

    if (use_optimization)
    {
      bspline_optimizers_.resize(10);
      for (int i = 0; i < 10; ++i)
      {
        bspline_optimizers_[i].reset(new BsplineOptimizer);
        bspline_optimizers_[i]->setParam(nh);
        bspline_optimizers_[i]->setEnvironment(edt_environment_);
      }
    }

  }

  bool FastPlannerManager::checkTrajCollision(double &distance)
  {
    double t_now = (ros::Time::now() - local_data_.start_time_).toSec();
    t_now = max(min(t_now, local_data_.duration_), 0.0);

    Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoorT(t_now);
    double radius = 0.0;
    Eigen::Vector3d fut_pt;
    double fut_t = 0.02;
    double t_inc = min(local_data_.duration_/10, 0.2);

    while (radius < 10.0 && t_now + fut_t < local_data_.duration_)
    {
      fut_pt = local_data_.position_traj_.evaluateDeBoorT(t_now + fut_t);
      // double dist = edt_environment_->sdf_map_->getDistance(fut_pt);
      if (sdf_map_->getInflateOccupancy(fut_pt) == 1)
      {
        if (sdf_map_->getInflateOccupancy(cur_pt) != 1 || sdf_map_->getOccupancy(fut_pt) == SDFMap::OCCUPIED)
        {
          distance = radius;
          return false;
        }
      }
      radius = (fut_pt - cur_pt).norm();
      fut_t += t_inc;
    }
    return true;
  }

  bool FastPlannerManager::checkPositionTraj()
  {
    double duration = local_data_.duration_;
    double t_cur = 0;
    double t_inc = min(duration/10, 0.2);
    while(t_cur + t_inc < duration)
    {
      Eigen::Vector3d pos_fut = local_data_.position_traj_.evaluateDeBoorT(t_cur + t_inc);
      Eigen::Vector3d pos_cur = local_data_.position_traj_.evaluateDeBoorT(t_cur);
      t_cur += t_inc;
      if (sdf_map_->getInflateOccupancy(pos_fut) == 1)
      {
        if (sdf_map_->getInflateOccupancy(pos_cur) != 1 || sdf_map_->getOccupancy(pos_fut) == SDFMap::OCCUPIED)
        {
          ROS_WARN("Trajectory Check Fail: Collision Detected!");
          return false;
        }
      }
    }
    return true;
  }

  bool FastPlannerManager::checkYawTraj()
  {
    double duration = local_data_.duration_;
    double t_cur = 0;
    double t_inc = 0.2;
    while (t_cur + t_inc < duration)
    {
      double yaw_cur = local_data_.yaw_traj_.evaluateDeBoorT(t_cur)[0];
      double yaw_fut = local_data_.yaw_traj_.evaluateDeBoorT(t_cur + t_inc)[0];
      double yaw_diff = yaw_fut - yaw_cur;
      t_cur += t_inc;
      if (abs(yaw_diff) / t_inc > pp_.max_yawdot_)
      {
        ROS_WARN(" Trajectory Check Fail: Yaw Rate Exceed!");
        return false;
      }
    }
    return true;
  }

  // SECTION kinodynamic replanning

  bool FastPlannerManager::kinodynamicReplan(const Eigen::Vector3d &start_pt,
                                             const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                             const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const double &time_lb)
  {
    std::cout << "[Kino replan]: start: " << start_pt.transpose() << ", " << start_vel.transpose()
              << ", "
              << start_acc.transpose() << ", goal:" << end_pt.transpose() << ", " << end_vel.transpose()
              << endl;

    if ((start_pt - end_pt).norm() < 1e-2)
    {
      cout << "Close goal" << endl;
      return false;
    }

    // Kinodynamic path searching

    kino_path_finder_->reset();
    int status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, true);
    if (status == KinodynamicAstar::NO_PATH)
    {
      ROS_INFO("search 1 fail");
      // Retry
      kino_path_finder_->reset();
      status = kino_path_finder_->search(start_pt, start_vel, start_acc, end_pt, end_vel, false);
      if (status == KinodynamicAstar::NO_PATH)
      {
        cout << "[Kino replan]: Can't find path." << endl;
        return false;
      }
    }
    plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

    // Parameterize path to B-spline
    double ts = pp_.ctrl_pt_dist / pp_.max_vel_;
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    kino_path_finder_->getSamples(ts, point_set, start_end_derivatives);

    // for (auto pt : point_set) std::cout << pt.transpose() << std::endl;
    // for (auto dr : start_end_derivatives) std::cout << dr.transpose() << std::endl;

    Eigen::MatrixXd ctrl_pts;
    NonUniformBspline::parameterizeToBspline(
        ts, point_set, start_end_derivatives, pp_.bspline_degree_, ctrl_pts);
    NonUniformBspline init(ctrl_pts, pp_.bspline_degree_, ts);

    // B-spline-based optimization
    int cost_function = BsplineOptimizer::NORMAL_PHASE;
    if (pp_.min_time_)
      cost_function |= BsplineOptimizer::MINTIME;
    vector<Eigen::Vector3d> start, end;
    init.getBoundaryStates(2, 0, start, end);
    bspline_optimizers_[0]->setBoundaryStates(start, end);
    if (time_lb > 0)
      bspline_optimizers_[0]->setTimeLowerBound(time_lb);

    // Swarm collision avoidance term
    // cost_function |= BsplineOptimizer::SWARM;
    // vector<NonUniformBspline> trajs;
    // swarm_traj_data_.getValidTrajs(trajs);
    // bspline_optimizers_[0]->setSwarmTrajs(trajs);

    bspline_optimizers_[0]->optimize(ctrl_pts, ts, cost_function, 1, 1);
    local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, ts);

    vector<Eigen::Vector3d> start2, end2;
    local_data_.position_traj_.getBoundaryStates(2, 0, start2, end2);
    //           << (start2[1] - start[1]).norm() << ", " << (start2[2] - start[2]).norm() << ")"
    //           << std::endl;

    updateTrajInfo();
    return true;
  }

  void FastPlannerManager::planExploreTraj(const vector<Eigen::Vector3d> &tour,
                                           const Eigen::Vector3d &cur_vel, const Eigen::Vector3d &cur_acc, const double &time_lb)
  {
    // Generate traj through waypoints-based method
    double dt;
    const int pt_num = tour.size();
    vector<Vector3d> points, boundary_deri;
    if (pt_num < 2)
    {
      ROS_ERROR("Empty path to traj planner");
    }
    else if (pt_num == 2)
    {
      double time = (tour[1] - tour[0]).norm() / (pp_.max_vel_ * 0.5);
      int seg_num = (tour[1] - tour[0]).norm() / pp_.ctrl_pt_dist;
      seg_num = max(2, seg_num);
      dt = time / double(seg_num);
      for (int i = 0; i <= seg_num; ++i)
      {
        Eigen::Vector3d pt = (1 - double(i) / seg_num) * tour[0] + double(i) / seg_num * tour[1];
        points.push_back(pt);
      }
      for (int i = 1; i <= 4; ++i)
      {
        boundary_deri.push_back(Eigen::Vector3d(0, 0, 0));
      }
    }
    else
    {
      Eigen::MatrixXd pos(pt_num, 3);
      for (int i = 0; i < pt_num; ++i)
        pos.row(i) = tour[i];

      Eigen::Vector3d zero(0, 0, 0);
      Eigen::VectorXd times(pt_num - 1);
      for (int i = 0; i < pt_num - 1; ++i)
        times(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);

      PolynomialTraj init_traj;
      PolynomialTraj::waypointsTraj(pos, cur_vel, zero, cur_acc, zero, times, init_traj);

      // B-spline-based optimization
      double duration = init_traj.getTotalTime();
      int seg_num = init_traj.getLength() / pp_.ctrl_pt_dist;
      seg_num = max(5, seg_num);
      dt = duration / double(seg_num);

      // std::endl;

      for (double ts = 0.0; ts <= duration + 1e-4; ts += dt)
        points.push_back(init_traj.evaluate(ts, 0));
      plan_data_.poly_path_ = points;
      boundary_deri.push_back(init_traj.evaluate(0.0, 1));
      boundary_deri.push_back(init_traj.evaluate(duration, 1));
      boundary_deri.push_back(init_traj.evaluate(0.0, 2));
      boundary_deri.push_back(init_traj.evaluate(duration, 2));
    }
    
    // adjust knot span
    dt = max(dt, time_lb/(points.size() - 1));

    Eigen::MatrixXd ctrl_pts;
    NonUniformBspline::parameterizeToBspline(
        dt, points, boundary_deri, pp_.bspline_degree_, ctrl_pts);
    NonUniformBspline tmp_traj(ctrl_pts, pp_.bspline_degree_, dt);
    local_data_.tmp_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt); // LZC added

    int cost_func = BsplineOptimizer::NORMAL_PHASE;
    if (pp_.min_time_)
      cost_func |= BsplineOptimizer::MINTIME;

    vector<Vector3d> start, end;
    tmp_traj.getBoundaryStates(2, 0, start, end);
    start[0] = tour[0];
    start[1] = cur_vel;
    start[2] = cur_acc;
    bspline_optimizers_[0]->setBoundaryStates(start, end);
    if (time_lb > 0)
      bspline_optimizers_[0]->setTimeLowerBound(time_lb);

    // Swarm collision avoidance term
    // cost_func |= BsplineOptimizer::SWARM;
    // vector<NonUniformBspline> trajs;
    // swarm_traj_data_.getValidTrajs(trajs);
    // bspline_optimizers_[0]->setSwarmTrajs(trajs);

    bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func, 1, 1);
    local_data_.position_traj_.setUniformBspline(ctrl_pts, pp_.bspline_degree_, dt);

    std::cout << "\033[44;37m origin traj duration = " << local_data_.tmp_traj_.duration_ << " \033[0m" << std::endl;
    std::cout << "\033[44;37m optimal traj duration = " << local_data_.position_traj_.duration_ << " \033[0m" << std::endl;
    std::cout << "\033[44;37m time lower bound = " << time_lb << " \033[0m" << std::endl;

    updateTrajInfo();
  }

  // only consider smoothness and boundary state
  void FastPlannerManager::planYawExplore(const Eigen::Vector3d &start_yaw,
                                          const double &end_yaw,
                                          bool lookfwd,
                                          const double &relax_time)
  {
    /********************* Calculate trajiecotry segment number ********************/
    double dt_yaw = 0.5;
    int seg_num = ceil(local_data_.duration_ / dt_yaw);
    seg_num = max(3, seg_num);  // Restrict the minimal num
    seg_num = min(12, seg_num); // Restrict the maximal num
    dt_yaw = local_data_.duration_ / double(seg_num);

    Eigen::Vector3d start_yaw3d = start_yaw;
    Eigen::Vector3d end_yaw3d(end_yaw, 0, 0);
    while (start_yaw3d[0] < -M_PI)
      start_yaw3d[0] += 2 * M_PI;
    while (start_yaw3d[0] > M_PI)
      start_yaw3d[0] -= 2 * M_PI;
    while (end_yaw3d[0] < -M_PI)
      end_yaw3d[0] += 2 * M_PI;
    while (end_yaw3d[0] > M_PI)
      end_yaw3d[0] -= 2 * M_PI;
    double last_yaw = start_yaw3d[0];
    calcNextYaw(last_yaw, end_yaw3d[0]);
    int last_idx = 0;
    std::cout << "***********************************************************" << std::endl;
    std::cout << "duration" << local_data_.duration_ 
              << ", dt_yaw: " << dt_yaw
              << ", start yaw: " << start_yaw3d[0]
              << ", end yaw: " << end_yaw3d[0] << std::endl;
    std::cout << "***********************************************************" << std::endl;

    // Debug rapid change of yaw
    if (fabs(start_yaw3d[0] - end_yaw3d[0]) >= M_PI)
    {
      ROS_WARN("Yaw change rapidly!");
    }

    /****************************** Generate waypoints *****************************/
    vector<Eigen::Vector3d> waypts;
    vector<int> waypt_idx;
    if (lookfwd)
    {
      // Add waypoint constraints if look forward is enabled
      const double forward_t = 2.0;
      const int relax_num2 = pp_.relax_time2_ / dt_yaw;
      for (int i = 1; i < seg_num - relax_num2; ++i)
      {
        // Check if the flight path segment is vertical
        double tc = i * dt_yaw;
        Eigen::Vector3d pc = local_data_.position_traj_.evaluateDeBoorT(tc);
        double tf = min(local_data_.duration_, tc + forward_t);
        Eigen::Vector3d pf = local_data_.position_traj_.evaluateDeBoorT(tf);
        Eigen::Vector3d pd = pf - pc;
        Eigen::Vector3d z1(0.0, 0.0, 1.0);
        Eigen::Vector3d z2(0.0, 0.0, -1.0);
        Eigen::Vector3d waypt;
        double theta_rad1 = std::acos(clamp((pd.dot(z1) / pd.norm()), -1.0, 1.0));
        double theta_rad2 = std::acos(clamp((pd.dot(z2) / pd.norm()), -1.0, 1.0));

        // Generate yaw waypoints
        if (pd.norm() > 1e-6 &&
            (fabs(theta_rad1) > M_PI / 6.0 && fabs(theta_rad2) > M_PI / 6.0))
        {
          waypt(0) = atan2(pd(1), pd(0));
          waypt(1) = waypt(2) = 0.0;
          calcNextYaw(last_yaw, waypt(0));

          // Check if the waypoint satisfy yawdot constraint
          double dt1 = double(i - last_idx) * dt_yaw;
          double dt2 = double(seg_num + 1 - i) * dt_yaw;

          double diff1 = fabs(waypt(0) - last_yaw);
          double yd1 = min(diff1, 2 * M_PI - diff1) / dt1;

          double tmp2 = end_yaw;
          calcNextYaw(waypt(0), tmp2);
          double diff2 = fabs(waypt(0) - tmp2);
          double yd2 = min(diff2, 2 * M_PI - diff2) / dt2;


          if (yd1 < pp_.max_yawdot_ && yd2 < pp_.max_yawdot_)
          {
            last_yaw = waypt(0);
            waypts.push_back(waypt);
            waypt_idx.push_back(i);
          }
        }
      }
      calcNextYaw(last_yaw, end_yaw3d[0]);
    }

    /*************************** Interpolate yaw on knots ****************************/
    int wp_adr = 0;
    vector<Eigen::Vector3d> points;
    Eigen::Vector3d start_pt_tmp = start_yaw3d;
    Eigen::Vector3d end_pt_tmp = end_yaw3d;
    if (waypts.size() > 0)
    {
      end_pt_tmp = waypts[0];
    }
    int start_idx_tmp = 0;
    int end_idx_tmp = seg_num;
    for (int i = 0; i <= seg_num; ++i)
    {
      if (waypts.size() > 0)
      {
        if (i == waypt_idx[wp_adr])
        {
          start_idx_tmp = waypt_idx[wp_adr];
          start_pt_tmp = waypts[wp_adr];
          wp_adr++;
          if (waypts.size() > wp_adr)
          {
            end_idx_tmp = waypt_idx[wp_adr];
            end_pt_tmp = waypts[wp_adr];
          }
          else
          {
            end_idx_tmp = seg_num;
            end_pt_tmp = end_yaw3d;
          }
        }
      }
      int interval_num = end_idx_tmp - start_idx_tmp;
      Eigen::Vector3d pt = (1 - double(i - start_idx_tmp) / interval_num) * start_pt_tmp + double(i - start_idx_tmp) / interval_num * end_pt_tmp;
      points.push_back(pt);
    }

    /*********************** Calculate yaw traj control points ***********************/
    Eigen::MatrixXd yaw_origin(seg_num + 3, 1);
    yaw_origin.setZero();
    vector<Vector3d> boundary_deri;
    for (int i = 1; i <= 4; ++i)
    {
      boundary_deri.push_back(Eigen::Vector3d(0, 0, 0));
    }
    // Generate traj through waypoints-based method
    NonUniformBspline yaw_traj_origin;
    NonUniformBspline::parameterizeToBspline(dt_yaw, points, boundary_deri, pp_.bspline_degree_, yaw_origin);
    yaw_traj_origin.setUniformBspline(yaw_origin, pp_.bspline_degree_, dt_yaw);

    // print yaw matrix

    /*********************** Optimize yaw traj control points ***********************/
    NonUniformBspline yaw_traj_optimal;
    Eigen::MatrixXd yaw_optimal = yaw_origin;
    // Call B-spline optimization solver
    int cost_func = BsplineOptimizer::SMOOTHNESS |
                    BsplineOptimizer::START |
                    BsplineOptimizer::END |
                    BsplineOptimizer::WAYPOINTS;
    vector<Eigen::Vector3d> start = {Eigen::Vector3d(start_yaw3d[0], 0, 0),
                                     Eigen::Vector3d(start_yaw3d[1], 0, 0), Eigen::Vector3d(start_yaw3d[2], 0, 0)};
    vector<Eigen::Vector3d> end = {Eigen::Vector3d(end_yaw3d[0], 0, 0),
                                   Eigen::Vector3d(0, 0, 0)};
    bspline_optimizers_[1]->setBoundaryStates(start, end);
    bspline_optimizers_[1]->setWaypoints(waypts, waypt_idx);
    bspline_optimizers_[1]->optimize(yaw_optimal, dt_yaw, cost_func, 1, 1);
    yaw_traj_optimal.setUniformBspline(yaw_optimal, pp_.bspline_degree_, dt_yaw);

    /******************************** Update traj info ******************************/
    // Find the better yaw trajectory
    double yaw_cost_origin = 0.0;
    double yaw_cost_optimal = 0.0;
    double yaw_end_optimal = yaw_traj_optimal.evaluateDeBoorT(seg_num * dt_yaw)[0];
    for(int i=1; i< seg_num + 3; i++)
    {
      yaw_cost_origin += fabs(yaw_origin(i) - yaw_origin(i-1));
      yaw_cost_optimal += fabs(yaw_optimal(i) - yaw_optimal(i-1));
    }

    // Set the better yaw trajectory
    if(yaw_cost_optimal < yaw_cost_origin && fabs(yaw_end_optimal- end_yaw3d[0]) < 0.1)
    {
      local_data_.yaw_traj_.setUniformBspline(yaw_optimal, 3, dt_yaw);
    }
    else
    {
      local_data_.yaw_traj_.setUniformBspline(yaw_origin, 3, dt_yaw);
    }
    local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
    local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();
    plan_data_.dt_yaw_ = dt_yaw;
  }

  void FastPlannerManager::calcNextYaw(const double &last_yaw, double &yaw)
  {
    // round yaw to [-PI, PI]
    double round_last = last_yaw;
    while (round_last < -M_PI)
    {
      round_last += 2 * M_PI;
    }
    while (round_last > M_PI)
    {
      round_last -= 2 * M_PI;
    }

    double diff = yaw - round_last;
    if (fabs(diff) <= M_PI)
    {
      yaw = last_yaw + diff;
    }
    else if (diff > M_PI)
    {
      yaw = last_yaw + diff - 2 * M_PI;
    }
    else if (diff < -M_PI)
    {
      yaw = last_yaw + diff + 2 * M_PI;
    }
  }

  void FastPlannerManager::updateTrajInfo()
  {
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }


} // namespace shield