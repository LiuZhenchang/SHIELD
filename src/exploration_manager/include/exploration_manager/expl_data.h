#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <Eigen/Eigen>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <bspline/Bspline.h>

using namespace std;
using Eigen::Vector3d;
using std::vector;

namespace shield
{
    struct FSMData
    {
        // FSM data
        bool trigger_, have_odom_, static_state_;
        vector<std::string> state_str_;

        Eigen::Vector3d odom_pos_, odom_vel_; // odometry state
        Eigen::Quaterniond odom_orient_;
        double odom_yaw_;

        Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_; // start state
        vector<Eigen::Vector3d> start_poss, hstory_path;
        bspline::Bspline newest_traj_;

        // Swarm collision avoidance
        bool avoid_collision_, go_back_;
        ros::Time fsm_init_time_;
        ros::Time last_check_frontier_time_;

        Eigen::Vector3d start_pos_;
        int record_count_ = 0;
        int plan_fail_count_ = 0;
        int odom_count_ = 0;
    };

    struct FSMParam
    {
        double replan_time_; // second
        double search_frt_interval_; // second
        int record_interval_ = 3;
        bool hgrid_enable_;
        bool return_home_;
        
        // Swarm
        // double attempt_interval_;  // Min interval of opt attempt
        // double pair_opt_interval_; // Min interval of successful pair opt
        // int repeat_send_num_;
    };

    struct DroneState
    {
        Eigen::Vector3d pos_;
        Eigen::Vector3d vel_;
        double yaw_;
        double stamp_; // Stamp of pos,vel,yaw

        vector<int> grid_ids_;     // Id of grid tour
        vector<int> ego_ids_;      // Id of grid tour
        vector<int> explored_ids_; // Id of grid tour
    };

    struct ExplorationData
    {
        vector<vector<Vector3d>> frontiers_;
        vector<vector<Vector3d>> dead_frontiers_;
        vector<Vector3d> averages_;
        vector<Vector3d> views_;
        vector<double> yaws_;
        vector<Vector3d> frontier_tour_;
        vector<vector<Vector3d>> other_tours_;
        vector<Vector3d> global_tour_;
        vector<double> global_yaw_;

        vector<int> refined_ids_;
        vector<vector<Vector3d>> n_points_;
        vector<Vector3d> unrefined_points_;
        vector<Vector3d> refined_points_;
        vector<Vector3d> refined_views_; // points + dir(yaw)
        vector<Vector3d> refined_views1_, refined_views2_;
        vector<Vector3d> refined_tour_;

        Vector3d next_goal_;
        vector<Vector3d> path_next_goal_, kino_path_;
        Vector3d next_pos_;
        double next_yaw_;
        bool explore_finish_; // LZC added


        // viewpoint planning
        // vector<Vector4d> views_;
        vector<Vector3d> views_vis1_, views_vis2_;
        vector<Vector3d> centers_, scales_;

        // Swarm, other drones' state
        vector<DroneState> swarm_state_;
        vector<double> pair_opt_stamps_, pair_opt_res_stamps_;
        vector<int> ego_ids_, other_ids_;
        vector<int> explored_ids_;        // LZC added
        vector<int> explored_ids_new;     // LZC added
        vector<int> explored_ids_prepare; // LZC added
        bool reallocated_, wait_response_;

        // Coverage planning
        vector<Vector3d> grid_tour_, grid_tour2_;
        // int prev_first_id_;
        // vector<int> last_grid_ids_;

        int current_explore_grid_id_;
        int plan_num_;
    };

    struct ExplorationParam
    {
        // params
        bool refine_local_;
        int refined_num_;
        double refined_radius_;
        int top_view_num_;
        double max_decay_;
        double traj_segment_length_;
        string tsp_dir_;  // resource dir of tsp solver
        string mtsp_dir_; // resource dir of tsp solver
        double relax_time_;
        int init_plan_num_;

        // Swarm
        int drone_num_;
        int drone_id_;
    };
}
#endif