#ifndef _SHIELD_EXPLORATION_FSM_H_
#define _SHIELD_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>
#include <thread>
#include <mutex> 

#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GridCells.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

#include <plan_env/sdf_map.h>
#include <bspline/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>
#include <plan_env/raycast.h>

#include <lidar/perception_utils_lidar.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace shield
{
    class FastPlannerManager;
    class FastExplorationManager;
    struct FSMParam;
    struct FSMData;

    enum EXPL_STATE
    {
        INIT,
        WAIT_TRIGGER,
        PLAN_TRAJ,
        PLAN_PATH,
        PUB_TRAJ,
        PUB_WAYPOINT,
        EXEC_TRAJ,
        FINISH,
        IDLE
    };

    class FastExplorationFSM
    {
    public:
        FastExplorationFSM(/* args */)
        {
        }
        ~FastExplorationFSM()
        {
            thread_running_ = false;
            if (callback_thread_.joinable()) {
                callback_thread_.join();
            }

            thread_odom_running_ = false;
            if (callback_odom_thread_.joinable()) {
                callback_odom_thread_.join();
            }
        }

        void init(ros::NodeHandle &nh);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        /* helper functions */
        int callExplorationPlanner();
        void transitState(EXPL_STATE new_state, string pos_call);
        int getId();

        /* ROS functions */
        void FSMCallback(const ros::TimerEvent &e);
        void safetyCallback(const ros::TimerEvent &e);
        void frontierCallback(const ros::TimerEvent &e);
        void triggerCallback(const geometry_msgs::PoseStampedConstPtr &msg);
        void odometryCallback(const nav_msgs::OdometryConstPtr &msg);
        void testCallback(const ros::TimerEvent &e);
        int findGlobalTourOfGrid(Vector3d &goal, double &yaw);


        void visualize();//visualize frontier
        void visualizeOdom();
        void visualizGlobalPath();
        void visualizeLocalTraj();
        void visualizeFrontier();
        void visualizeEnvironment();
        void visualizeUpdateBox();



        // Swarm
        void droneStateTimerCallback(const ros::TimerEvent &e);
        void droneStateMsgCallback(const nav_msgs::GridCellsConstPtr &msg);

        /* planning utils */
        shared_ptr<FastPlannerManager> planner_manager_;
        shared_ptr<FastExplorationManager> expl_manager_;
        PlanningVisualization::Ptr visualization_;
        shared_ptr<FSMParam> fp_;
        shared_ptr<FSMData> fd_;
        EXPL_STATE state_;

        /* ROS utils */
        ros::NodeHandle node_;
        ros::Timer exec_timer_, safety_timer_, frontier_timer_, search_timer_;
        ros::Subscriber trigger_sub_, odom_sub_;
        ros::Publisher replan_pub_, new_pub_, bspline_pub_;
        std_msgs::Int16 replan_msg_; //LZC Added
        
        ros::CallbackQueue custom_queue_;  // custom callback queue
        boost::thread callback_thread_;    // thread that processes the queue
        bool thread_running_;              // thread running flag

        ros::CallbackQueue custom_odom_queue_;  // custom callback queue
        boost::thread callback_odom_thread_;    // thread that processes the queue
        bool thread_odom_running_;

        // thread execution function
        void threadFunc();

        void threadOdomFunc();

        //print thread id
        void printThreadId(const std::string& context);
        void searchFrontiersCallback(const ros::TimerEvent &e);
        bool start_searchfrontiers_flag_ = false;
        std::mutex searchfrontiers_mutex_; // standard C++ mutex, protects frontier search operations

        // Swarm state
        // ros::Publisher drone_state_pub_;
        // ros::Subscriber drone_state_sub_;
        // ros::Timer drone_state_timer_;

        // unique_ptr<RayCaster> raycaster_;
        

        /******************LZC added program******************/
        // double frame_type_;
        // double path_length_;
        // ros::Time t_past_;
        // ros::Time t_curr_;
        // ros::Time t_post_;
        // bool tour_thread_running_;
        // ros::Publisher waypoint_pub_;
        // ros::Publisher path_pub_;
        // int id_share_length_ego = 1;
        // int id_share_length_exp = 2;
        // nav_msgs::Path uav_path_;
        // void PathCallback(const ros::TimerEvent &e);
        /*****************************************************/

        // vector<Vector3d> global_tour;
        // vector<double> global_yaw;
        // vector<Vector3d> tsp_tour;

        std::string frame_id_;
        double marker_scale_; // scale factor for visualization markers

    };
}

#endif