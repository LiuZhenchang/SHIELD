#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <bspline_opt/bspline_optimizer.h>
#include <bspline/non_uniform_bspline.h>

#include <path_searching/astar2.h>
#include <path_searching/kinodynamic_astar.h>

#include <plan_env/edt_environment.h>

#include <plan_manage/plan_container.hpp>

#include <ros/ros.h>

namespace shield
{
  // Fast Planner Manager
  // Key algorithms of mapping and planning are called

  class FastPlannerManager
  {
    // SECTION stable
  public:
    FastPlannerManager();
    ~FastPlannerManager();

    /* main planning interface */
    bool kinodynamicReplan( const Eigen::Vector3d &start_pt, 
                            const Eigen::Vector3d &start_vel,
                            const Eigen::Vector3d &start_acc,
                            const Eigen::Vector3d &end_pt,
                            const Eigen::Vector3d &end_vel, 
                            const double &time_lb = -1);

    void planExploreTraj( const vector<Eigen::Vector3d> &tour, 
                          const Eigen::Vector3d &cur_vel,
                          const Eigen::Vector3d &cur_acc, 
                          const double &time_lb = -1);

    void planYawExplore(const Eigen::Vector3d &start_yaw, 
                        const double &end_yaw, 
                        bool lookfwd,
                        const double &relax_time);

    void initPlanModules(ros::NodeHandle &nh);

    bool checkTrajCollision(double &distance);
    bool checkPositionTraj();
    bool checkYawTraj();
    void calcNextYaw(const double &last_yaw, double &yaw);

    PlanParameters pp_;
    LocalTrajData local_data_;
    MidPlanData plan_data_;
    EDTEnvironment::Ptr edt_environment_;
    unique_ptr<Astar> path_finder_;
    unique_ptr<KinodynamicAstar> kino_path_finder_;

    /********************** LZC Added ***********************/
    // enum
    // {
    //   SOLUTION_FAIL = 0,
    //   GENERATE_WAYPOINT = 1,
    //   GENERATE_PATH = 2,
    //   DEACTIVE_GOAL = 3
    // };
    // Eigen::Vector3d waypoint_;
    // Eigen::Vector3d waypoint_last_;
    // Eigen::Vector3d pos_safe_;
    // Eigen::Vector3d goal_;
    // vector<Eigen::Vector3d> path_;

    /********************** LZC Added ***********************/

  private:
    /* main planning algorithms & modules */

    shared_ptr<SDFMap> sdf_map_;
    vector<BsplineOptimizer::Ptr> bspline_optimizers_;

    void updateTrajInfo();
  };
} // namespace shield

#endif