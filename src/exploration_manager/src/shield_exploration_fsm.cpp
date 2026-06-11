
#include <exploration_manager/shield_exploration_fsm.h>
#include <exploration_manager/shield_exploration_manager.h>
#include <exploration_manager/expl_data.h>
#include <active_perception/hgrid.h>
#include <active_perception/frontier_finder.h>
#include <fstream>
#include <random>

using namespace std;
using Eigen::Vector4d;
namespace shield
{
    void FastExplorationFSM::init(ros::NodeHandle &nh)
    {
        fp_.reset(new FSMParam);
        fd_.reset(new FSMData);

        nh.param("fsm/replan_time", fp_->replan_time_, -1.0);
        nh.param("fsm/hgrid_enable", fp_->hgrid_enable_, false);
        nh.param("fsm/return_home", fp_->return_home_, false);
        nh.param("fsm/search_frt_interval", fp_->search_frt_interval_, 2.0);
        nh.param("fsm/frame_id", frame_id_, string("world"));
        nh.param("general/marker_scale", marker_scale_, 0.2);

        fd_->have_odom_ = false;
        fd_->state_str_ = {"INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PLAN_PATH", "PUB_TRAJ", "PUB_WAYPOINT", "EXEC_TRAJ", "FINISH", "IDLE"};
        fd_->static_state_ = true;
        fd_->trigger_ = false;
        fd_->avoid_collision_ = false;
        fd_->go_back_ = false;
        fd_->odom_pos_ = {0, 0, 0}; // LZC added

        /* Initialize main modules */
        expl_manager_.reset(new FastExplorationManager);
        expl_manager_->initialize(nh);
        visualization_.reset(new PlanningVisualization(nh));

        planner_manager_ = expl_manager_->planner_manager_;
        state_ = EXPL_STATE::INIT;

        /* Ros sub, pub and timer */
        exec_timer_ = nh.createTimer(ros::Duration(0.1), &FastExplorationFSM::FSMCallback, this);
        safety_timer_ = nh.createTimer(ros::Duration(0.1), &FastExplorationFSM::safetyCallback, this);
        frontier_timer_ = nh.createTimer(ros::Duration(1.0), &FastExplorationFSM::frontierCallback, this);
        trigger_sub_ = nh.subscribe("/move_base_simple/goal", 1, &FastExplorationFSM::triggerCallback, this);

        replan_pub_ = nh.advertise<std_msgs::Int16>("/planning/replan", 10);
        new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
        bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);

        // search_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::searchFrontiersCallback, this);
        // Key change: use a NodeHandle with a custom queue
        ros::NodeHandle nh_custom;
        nh_custom.setCallbackQueue(&custom_queue_);
        search_timer_ = nh_custom.createTimer(ros::Duration(fp_->search_frt_interval_), &FastExplorationFSM::searchFrontiersCallback, this);
        // Start the thread that processes the custom queue
        thread_running_ = true;
        callback_thread_ = boost::thread(&FastExplorationFSM::threadFunc, this);

        ros::NodeHandle nh_odom_custom;
        nh_odom_custom.setCallbackQueue(&custom_odom_queue_);
        odom_sub_ = nh_odom_custom.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);

        // Start the thread that processes the custom queue
        thread_odom_running_ = true;
        callback_odom_thread_ = boost::thread(&FastExplorationFSM::threadOdomFunc, this);

    }

    void FastExplorationFSM::frontierCallback(const ros::TimerEvent &e)
    {
        if (state_ == EXPL_STATE::WAIT_TRIGGER || state_ == EXPL_STATE::IDLE || state_ == EXPL_STATE::FINISH)
        {
            if (!fp_->hgrid_enable_)
            {
                /********frt tsp******* */
                std::lock_guard<std::mutex> lock(searchfrontiers_mutex_);
                int res1 = expl_manager_->planExploreMotion(fd_->odom_pos_, fd_->odom_vel_, Vector3d(0, 0, 0), Vector3d(fd_->odom_yaw_, 0, 0));
                visualizGlobalPath();
                visualizeFrontier();
                start_searchfrontiers_flag_ = true; // mark that searchfrontiers can begin
                // restart exploration if new frontier is found
                if (res1 == 2 && state_ != EXPL_STATE::WAIT_TRIGGER)
                {
                    fd_->go_back_ = false;
                    fd_->static_state_ = true;
                    transitState(PLAN_TRAJ, "frontierCallback");
                }
            }
            else
            {
                /**********CP******** */
                std::lock_guard<std::mutex> lock(searchfrontiers_mutex_);
                expl_manager_->frontier_finder_->searchFrontiers(fd_->odom_pos_, fd_->odom_yaw_);
                expl_manager_->frontier_finder_->computeFrontiersToVisit(fd_->odom_pos_);
                expl_manager_->frontier_finder_->updateFrontierCostMatrix();
                // cp change: put the frts found by frontier_finder into expl_manager_'s ed_->averages_
                expl_manager_->frontier_finder_->getTopViewpointsInfo(fd_->odom_pos_, expl_manager_->ed_->views_, expl_manager_->ed_->yaws_, expl_manager_->ed_->averages_);
                visualizeFrontier();
                Eigen::Vector3d goal;
                double yaw;
                int res_cp = findGlobalTourOfGrid(goal, yaw);
                start_searchfrontiers_flag_ = true; // mark that searchfrontiers can begin

                if ((res_cp == 0 || res_cp == 1) && state_ != EXPL_STATE::WAIT_TRIGGER)
                {
                    expl_manager_->ed_->next_pos_ = goal;
                    expl_manager_->ed_->next_yaw_ = yaw;
                    fd_->go_back_ = false;
                    fd_->static_state_ = true;
                    transitState(PLAN_TRAJ, "frontierCallback");
                }
                /**********CP******** */
            }
        }
        else
        {
            // expl_manager_->frontier_finder_->searchFrontiers(fd_->odom_pos_, fd_->odom_yaw_);
            // expl_manager_->frontier_finder_->computeFrontiersToVisit(fd_->odom_pos_);
            // expl_manager_->frontier_finder_->updateFrontierCostMatrix();
        }
        visualizeEnvironment();
        visualizeUpdateBox();
    }

    void FastExplorationFSM::threadFunc()
    {
        // printThreadId("threadFunc");
        ros::Rate rate(500); // high-frequency processing
        while (thread_running_ && ros::ok())
        {
            custom_queue_.callAvailable(ros::WallDuration(0.01));
            rate.sleep();
        }
    }

    void FastExplorationFSM::threadOdomFunc()
    {
        // printThreadId("threadOdomFunc");
        ros::Rate rate(500); // high-frequency processing
        while (thread_odom_running_ && ros::ok())
        {
            custom_odom_queue_.callAvailable(ros::WallDuration(0.01));
            rate.sleep();
        }
    }

    void FastExplorationFSM::printThreadId(const std::string &context)
    {
        std::thread::id this_id = std::this_thread::get_id();
        std::string showText = "\033[44;37m" + context + " running in thread: " + "\033[0m";
    }

    void FastExplorationFSM::searchFrontiersCallback(const ros::TimerEvent &e)
    {
        printThreadId("searchFrontiers");
        ROS_INFO_STREAM_THROTTLE(
            1, "[FSM]: Drone " << getId() << " state: " << fd_->state_str_[int(state_)]);

        if (state_ == EXPL_STATE::EXEC_TRAJ)
        {
            std::lock_guard<std::mutex> lock(searchfrontiers_mutex_);
            expl_manager_->frontier_finder_->searchFrontiers(fd_->odom_pos_, fd_->odom_yaw_);
            expl_manager_->frontier_finder_->computeFrontiersToVisit(fd_->odom_pos_);
            expl_manager_->frontier_finder_->updateFrontierCostMatrix();
            // cp change: put the frts found by frontier_finder into expl_manager_'s ed_->averages_
            expl_manager_->frontier_finder_->getTopViewpointsInfo(fd_->odom_pos_, expl_manager_->ed_->views_, expl_manager_->ed_->yaws_, expl_manager_->ed_->averages_);
            if (!expl_manager_->frontier_finder_->haveTargetFrontier())
            {
                fd_->static_state_ = false;
                transitState(PLAN_TRAJ, "searchFrontiersCallback");
                ROS_INFO_STREAM("\033[45;37m PLAN_TRAJ - target frontier changed \033[0m");
            }
            visualizeFrontier();
        }
    }

    void FastExplorationFSM::safetyCallback(const ros::TimerEvent &e)
    {

        if (state_ == EXPL_STATE::INIT ||
            state_ == EXPL_STATE::WAIT_TRIGGER ||
            state_ == EXPL_STATE::IDLE ||
            state_ == EXPL_STATE::FINISH)
            return;

        LocalTrajData *info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
        t_r = max(min(t_r, info->duration_), 0.0);
        Eigen::Vector3d goal_pt = info->position_traj_.evaluateDeBoorT(t_r);
        Vector3d dir = (goal_pt - fd_->odom_pos_).normalized();
        double len = (goal_pt - fd_->odom_pos_).norm();

        if (state_ == EXPL_STATE::EXEC_TRAJ)
        {
            // Check wether the trajectory target point is far away from the current position
            // if (len > 3.0)
            // {
            //     fd_->static_state_ = true;
            //     replan_msg_.data = 1;
            //     transitState(PLAN_TRAJ, "safetyCallback");
            //     return;
            // }

            // Check whether trajectory is collide with inflate obstacles
            double dist;
            bool safe = planner_manager_->checkTrajCollision(dist);
            if (!safe)
            {
                ROS_WARN(" SAFETYCALLBACK REPLANE: trajectory collision detected ");
                fd_->avoid_collision_ = true;
                fd_->static_state_ = false;
                replan_msg_.data = 1;
                std::cout << "state_ = " << state_ << std::endl;
                transitState(PLAN_TRAJ, "safetyCallback");
                return;
            }

            // Check whether goal is in the inflate obstacles
            if (planner_manager_->edt_environment_->sdf_map_->getInflateOccupancy(expl_manager_->ed_->next_pos_) == 1)
            {
                ROS_WARN(" SAFETYCALLBACK REPLANE: invalid target ");
                fd_->static_state_ = true;
                replan_msg_.data = 1;
                ROS_INFO_STREAM("\033[43;37m deactive target frontier inflate occupancy \033[0m");
                expl_manager_->frontier_finder_->deactiveTargetFrontier(expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_);
                std::cout << "state_ = " << state_ << std::endl;
                transitState(PLAN_TRAJ, "safetyCallback");
                return;
            }

            // Check wether obstacles exist between UAV and current goal
            if (fd_->static_state_ == false)
            {
                double len_inc = min(len / 5, 0.1);
                for (double l = len_inc; l <= len + 1e-2; l += len_inc)
                {
                    Vector3d ckpt = fd_->odom_pos_ + l * dir;
                    if (planner_manager_->edt_environment_->sdf_map_->getOccupancy(ckpt) == SDFMap::OCCUPIED)
                    {
                        ROS_WARN("SAFETYCALLBACK REPLANE: movement collision detected ");
                        fd_->avoid_collision_ = true;
                        fd_->static_state_ = true;
                        replan_msg_.data = 1;
                        std::cout << "state_ = " << state_ << std::endl;
                        transitState(PLAN_TRAJ, "safetyCallback");
                        return;
                    }
                }
            }
        }
    }

    void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        printThreadId("odometryCallback");
        // reduce odom message process frequency to avoid callback queue full
        fd_->odom_count_++;
        if (fd_->odom_count_ < 3)
        {
            return;
        }

        fd_->odom_count_ = 0;

        // store odometry data
        fd_->odom_pos_(0) = msg->pose.pose.position.x;
        fd_->odom_pos_(1) = msg->pose.pose.position.y;
        fd_->odom_pos_(2) = msg->pose.pose.position.z;

        expl_manager_->hgrid_->setDronePos(fd_->odom_pos_); // set the Hgrid position the drone is located in

        fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
        fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
        fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
        fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

        // Convert velocity from body frame to world frame
        Eigen::Vector3d vel_odom;
        vel_odom(0) = msg->twist.twist.linear.x;
        vel_odom(1) = msg->twist.twist.linear.y;
        vel_odom(2) = msg->twist.twist.linear.z;
        
        // in marsim, odom vel is in world frame
        fd_->odom_vel_ = vel_odom;
  
        Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
        fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

        if (!fd_->have_odom_)
        {
            fd_->have_odom_ = true;
            fd_->start_pos_ = fd_->odom_pos_;
            fd_->fsm_init_time_ = ros::Time::now();
            // t_past_ == ros::Time::now();
            // t_post_ == ros::Time::now();
        }
        visualizeOdom();
    }

    int FastExplorationFSM::getId()
    {
        return expl_manager_->ep_->drone_id_;
    }

    void FastExplorationFSM::FSMCallback(const ros::TimerEvent &e)
    {
        ROS_INFO_STREAM_THROTTLE(
            1, "[FSM]: Drone " << getId() << " state: " << fd_->state_str_[int(state_)]);

        switch (state_)
        {
        case INIT:
        {
            // Wait for odometry ready
            if (!fd_->have_odom_)
            {
                ROS_WARN_THROTTLE(1.0, "no odom");
                return;
            }
            if ((ros::Time::now() - fd_->fsm_init_time_).toSec() < 2.0)
            {
                ROS_WARN_THROTTLE(1.0, "wait for init");
                return;
            }
            // Go to wait trigger when odom is ok
            transitState(WAIT_TRIGGER, "FSM");
            break;
        }
        case WAIT_TRIGGER:
        {
            // Do nothing but wait for trigger
            ROS_WARN_THROTTLE(1.0, "wait for trigger.");
            break;
        }
        case IDLE:
        {
            double check_interval = (ros::Time::now() - fd_->last_check_frontier_time_).toSec();
            if (check_interval > 1.0)
            {
                ROS_WARN("====================== Go back to start point ======================");
                Eigen::Vector3d bmin, bmax;
                expl_manager_->sdf_map_->getBox(bmin, bmax);
                expl_manager_->ed_->next_pos_ = fd_->start_pos_;
                expl_manager_->ed_->next_pos_.z() = max(min(6.0, (bmax.z() + bmin.z()) / 2), 1.0);
                expl_manager_->ed_->next_yaw_ = 0.0;

                if (fp_->return_home_)
                {
                    ROS_WARN("Return to home position");
                    fd_->go_back_ = true;
                    replan_msg_.data = 0;
                    transitState(PLAN_TRAJ, "FSM_IDLE");
                }
                else
                {
                    ROS_WARN("No need to return to home position, finish exploration");
                    expl_manager_->ed_->explore_finish_ = true;
                    transitState(FINISH, "FSM_IDLE");
                    break;
                }
            }
            break;
        }
        case FINISH:
        {
            ROS_WARN("Finish exploration");
            expl_manager_->ed_->explore_finish_ = true;
            expl_manager_->sdf_map_->savePointCloud();
            break;
        }
        case PLAN_TRAJ:
        {
            ROS_WARN_THROTTLE(0.1, "Replan Trajectory");
            LocalTrajData *info = &planner_manager_->local_data_;

            if (!fp_->hgrid_enable_)
            {
                /*************************** frt tsp *************************/
                std::lock_guard<std::mutex> lock(searchfrontiers_mutex_);
                int res1 = expl_manager_->planExploreMotion(fd_->odom_pos_, fd_->odom_vel_, Vector3d(0, 0, 0), Vector3d(fd_->odom_yaw_, 0, 0));
                if (res1 == 2) // plan success
                {
                    visualizGlobalPath();
                    visualizeFrontier();
                    fd_->go_back_ = false;
                }
                else
                {
                    ROS_WARN_THROTTLE(0.1, "PLAN_TRAJ: No global tour");
                    if (!fd_->go_back_)
                    {
                        transitState(IDLE, "FSM");
                        break;
                    }
                }
            }
            else
            {
                /*************************** CP *************************/
                std::lock_guard<std::mutex> lock(searchfrontiers_mutex_);

                struct timeval t1, t2, t3, t4, t5;
                gettimeofday(&t1, NULL);

                expl_manager_->frontier_finder_->searchFrontiers(fd_->odom_pos_, fd_->odom_yaw_);

                gettimeofday(&t2, NULL);
                long seconds = t2.tv_sec - t1.tv_sec;
                long usec = t2.tv_usec - t1.tv_usec;
                double elapsed = seconds + usec / 1e6;

                ROS_INFO_STREAM("\033[33m searchFrontiers time" << elapsed << " \033[0m");

                expl_manager_->frontier_finder_->computeFrontiersToVisit(fd_->odom_pos_);

                gettimeofday(&t3, NULL);
                seconds = t3.tv_sec - t2.tv_sec;
                usec = t3.tv_usec - t2.tv_usec;
                elapsed = seconds + usec / 1e6;

                ROS_INFO_STREAM("\033[33m computeFrontiersToVisit time" << elapsed << " \033[0m");

                expl_manager_->frontier_finder_->updateFrontierCostMatrix();

                gettimeofday(&t4, NULL);
                seconds = t4.tv_sec - t3.tv_sec;
                usec = t4.tv_usec - t3.tv_usec;
                elapsed = seconds + usec / 1e6;

                ROS_INFO_STREAM("\033[33m updateFrontierCostMatrix time" << elapsed << " \033[0m");

                // cp change: put the frts found by frontier_finder into expl_manager_'s ed_->averages_
                expl_manager_->frontier_finder_->getTopViewpointsInfo(fd_->odom_pos_, expl_manager_->ed_->views_, expl_manager_->ed_->yaws_, expl_manager_->ed_->averages_);

                gettimeofday(&t5, NULL);
                seconds = t5.tv_sec - t4.tv_sec;
                usec = t5.tv_usec - t4.tv_usec;
                elapsed = seconds + usec / 1e6;

                ROS_INFO_STREAM("\033[33m getTopViewpointsInfo time" << elapsed << " \033[0m");

                visualizeFrontier();

                Eigen::Vector3d goal;
                double yaw;
                int res_cp = findGlobalTourOfGrid(goal, yaw);

                if (res_cp == 0)
                {
                    expl_manager_->ed_->next_pos_ = goal;
                    expl_manager_->ed_->next_yaw_ = yaw;
                    fd_->go_back_ = false;
                }
                else
                {
                    ROS_WARN_THROTTLE(0.1, "PLAN_TRAJ: No global tour");
                    if (!fd_->go_back_)
                    {
                        transitState(IDLE, "FSM");
                        break;
                    }
                }
            }
            /*************************** plan local trajectory *************************/
            // get initial state
            // if (fd_->static_state_ && fd_->odom_vel_.norm() < 0.3)
            if (fd_->static_state_)
            {
                if (fd_->avoid_collision_ || info->traj_id_ == 0)
                {
                    // Plan from static state (hover)
                    fd_->start_pt_ = fd_->odom_pos_;
                    fd_->start_yaw_(0) = fd_->odom_yaw_;
                }
                else
                {
                    double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
                    t_r = max(min(t_r, info->duration_), 0.0);
                    fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
                    fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
                }

                // fd_->start_vel_ = fd_->odom_vel_;
                fd_->start_vel_.setZero();
                fd_->start_acc_.setZero();
                fd_->start_yaw_(1) = 0.0;
                fd_->start_yaw_(2) = 0.0;
                ROS_WARN_THROTTLE(0.1, "PLAN_TRAJ: Replan in Static condition");
            }
            else
            {
                // Replan from non-static state, starting from 'replan_time' seconds later
                double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;
                t_r = max(min(t_r, info->duration_), 0.0);
                fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
                fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
                fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
                fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
                fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
                fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
            }
            // replan trajectory
            replan_pub_.publish(replan_msg_);
            int res = callExplorationPlanner();
            if (res == SUCCEED)
            {
                fd_->avoid_collision_ = false;
                fd_->plan_fail_count_ = 0;
                transitState(PUB_TRAJ, "FSM");
            }
            else
            {
                fd_->static_state_ = true;
                fd_->avoid_collision_ = true;
                fd_->plan_fail_count_++;
                ROS_WARN("PLAN_TRAJ: Plan fail");

                if (fd_->plan_fail_count_ > 3)
                {
                    ROS_INFO_STREAM("\033[43;37m deactive target frontier plan fail \033[0m");
                    expl_manager_->frontier_finder_->deactiveTargetFrontier(expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_);
                }
            }
            break;
        }

        case PUB_TRAJ:
        {
            double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
            if (dt > 0)
            {
                bspline_pub_.publish(fd_->newest_traj_);
                fd_->static_state_ = false;
                transitState(EXEC_TRAJ, "FSM");
            }
            break;
        }
        case EXEC_TRAJ:
        {
            /* determine if need to replan */
            LocalTrajData *info = &planner_manager_->local_data_;
            ros::Time time_now = ros::Time::now();
            double t_cur = (time_now - info->start_time_).toSec();
            Eigen::Vector3d traj_end_pt = info->position_traj_.evaluateDeBoorT(info->duration_);
            if (!fd_->go_back_)
            {
                // if (info->duration_ - t_cur < fp_->replan_time_ &&
                //     ((traj_end_pt - expl_manager_->ed_->next_pos_).norm() > 0.2 ||
                //         fabs(fd_->odom_yaw_ - expl_manager_->ed_->next_yaw_) > 0.1))
                if (info->duration_ - t_cur < fp_->replan_time_)
                {
                    /* add this block of code to switch to point-to-point flight */
                    // if((fd_->odom_pos_ - expl_manager_->ed_->next_pos_).norm() < 0.2){
                    //     transitState(FINISH, "FSM_EXEC_TRAJ");
                    //     break;
                    // }
                    /* add this block of code to switch to point-to-point flight */
                    // Replan if traj is almost fully executed
                    ROS_WARN("=================== Replan: explore traj fully executed ===================");
                    fd_->static_state_ = false;
                    replan_msg_.data = 0;
                    transitState(PLAN_TRAJ, "FSM_EXEC_TRAJ");
                }
            }
            else
            {
                // Check if reach goal
                if ((fd_->odom_pos_ - expl_manager_->ed_->next_pos_).norm() < 0.5)
                {
                    transitState(FINISH, "FSM_EXEC_TRAJ");
                    return;
                }
                if (info->duration_ - t_cur < fp_->replan_time_)
                {
                    // Replan for going back
                    ROS_WARN("==================== Replan: go back traj fully executed ==================");
                    fd_->static_state_ = false;
                    replan_msg_.data = 0;
                    transitState(PLAN_TRAJ, "FSM_EXEC_TRAJ");
                }
            }
            break;
        }
        }
    }

    void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call)
    {
        int pre_s = int(state_);
        state_ = new_state;
        ROS_INFO_STREAM("[" + pos_call + "]: Drone "
                        << getId()
                        << " from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]);
    }

    // cp change: in the fsm, call the cp-related computation functions and visualize
    int FastExplorationFSM::findGlobalTourOfGrid(Vector3d &goal, double &yaw)
    {

        vector<int> grid_id, frt_id;

        bool status = expl_manager_->findGlobalTourOfGrid(
            {fd_->odom_pos_}, {fd_->odom_vel_}, grid_id, true);

        // draw the hgrid grid
        vector<int> ids, ids_free, ids_unknown;
        vector<Eigen::Vector3d> centers;
        vector<Eigen::Vector3d> centers_free;
        vector<Eigen::Vector3d> centers_unknown;
        Eigen::Vector3d grid_size = expl_manager_->hgrid_->getResolution();

        expl_manager_->hgrid_->getCenterAll(centers, ids, 0);                 // get the geometric center
        expl_manager_->hgrid_->getCenterAll(centers_free, ids_free, 1);       // get the weighted center of free voxels
        expl_manager_->hgrid_->getCenterAll(centers_unknown, ids_unknown, 2); // get the weighted center of unknown voxels

        visualization_->drawHgridBox(centers, grid_size);
        visualization_->displaySphereList(centers_free, marker_scale_, Eigen::Vector4d(0, 0, 1, 1), 2, 7);
        visualization_->displaySphereList(centers_unknown, marker_scale_, Eigen::Vector4d(1, 0, 1, 1), 3, 7);

        // determine which grid the odom is in
        // int id = expl_manager_->hgrid_->getGridIDofPos(fd_->odom_pos_);
        // visualization_->drawText(fd_->odom_pos_, std::to_string(id), 2, Eigen::Vector4d(0.5, 0.5, 0.5, 1), "text", 10086, 7);

        for (int i = 0; i < ids.size(); i++)
        {
            string str;
            int frt_num = expl_manager_->hgrid_->getFrontierNum(ids[i]);
            int type = expl_manager_->hgrid_->getHGridStatus(ids[i]);
            double unknown_percent = expl_manager_->hgrid_->get_unknown_percent(ids[i]);
            switch (type)
            {
            case 0:
                str = "ACTIVE";
                break;
            case 1:
                str = "DORMANT";
                break;
            default:
                str = "DEACTIVE";
                break;
            }

            visualization_->drawText(centers[i], std::to_string(ids[i]) + " " + std::to_string(unknown_percent) + " " + std::to_string(frt_num) + " " + str, marker_scale_, Eigen::Vector4d(0.5, 0.5, 0.5, 1), "text", i, 7);
        }

        vector<Eigen::Vector3d> centers_all;
        centers_all.clear();
        centers_all.insert(centers_all.end(), centers_free.begin(), centers_free.end());
        centers_all.insert(centers_all.end(), centers_unknown.begin(), centers_unknown.end());

        vector<int> ids_all;
        ids_all.insert(ids_all.end(), ids_free.begin(), ids_free.end());
        ids_all.insert(ids_all.end(), ids_unknown.begin(), ids_unknown.end());

        vector<Eigen::Vector3d> hgrid_path_;

        vector<Frontier> frontiers;
        expl_manager_->frontier_finder_->mygetFrontiers(frontiers);

        if (!grid_id.empty())
        {

            expl_manager_->findGridAndFrontierPath(fd_->odom_pos_, fd_->odom_vel_, fd_->odom_yaw_, grid_id, frt_id);
            hgrid_path_.push_back(fd_->odom_pos_);
            for (int i = 0; i < frt_id.size(); i++)
            {
                hgrid_path_.push_back(frontiers[frt_id[i]].viewpoints_[0].pos_);
            }
            if (grid_id.size() >= 2)
            {
                for (int i = 1; i < grid_id.size(); i++)
                {
                    hgrid_path_.push_back(expl_manager_->hgrid_->getCenter(grid_id[i], 1));
                }
            }
            visualization_->drawHGridPath(hgrid_path_);
        }
        if (!frt_id.empty())
        {
            // a valid frt_id exists, publish it
            expl_manager_->frontier_finder_->setTargetFrontier(frt_id[0]);
            goal = frontiers[frt_id[0]].viewpoints_[0].pos_;
            yaw = frontiers[frt_id[0]].viewpoints_[0].yaw_;
            return 0;
        }
        else
        {
            return 1;
        }

        // if(!grid_id.empty()){
        //     goal = expl_manager_->hgrid_->getCenter(grid_id[0], 2);
        //     yaw = 0;
        //     expl_manager_->hgrid_->dormantGrid(grid_id[0]);
        //     return 1; //No frontier in grid
        // }
        // else{
        //     return 2; //no grid
        // }
    }

    void FastExplorationFSM::triggerCallback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        // if (state_ != WAIT_TRIGGER)
        //     return;
        fd_->trigger_ = true;
        cout << "Triggered!" << endl;
        fd_->start_pos_ = fd_->odom_pos_;
        ROS_WARN_STREAM("Start expl pos: " << fd_->start_pos_.transpose());
        replan_msg_.data = 0;

        // Eigen::Quaterniond quaternion(
        //     msg->pose.orientation.w,
        //     msg->pose.orientation.x,
        //     msg->pose.orientation.y,
        //     msg->pose.orientation.z);
        // Eigen::Matrix3d rotationMatrix = quaternion.toRotationMatrix();
        // Eigen::Vector3d eulerAngles = rotationMatrix.eulerAngles(2, 1, 0);
        // double yaw = eulerAngles(0);
        // Vector3d next_pos;
        // next_pos << msg->pose.position.x, msg->pose.position.y, 1.0;
        // expl_manager_->ed_->next_pos_ = next_pos;
        // expl_manager_->ed_->next_yaw_ = M_PI_2;
        ROS_WARN("========================== Replan: trigger ===========================");
        transitState(PLAN_TRAJ, "triggerCallback");
    }

    int FastExplorationFSM::callExplorationPlanner()
    {
        ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

        int res;

        res = expl_manager_->planTrajToView(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                            fd_->start_yaw_, expl_manager_->ed_->next_pos_, expl_manager_->ed_->next_yaw_);

        if (res == SUCCEED)
        {
            auto info = &planner_manager_->local_data_;
            info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

            bspline::Bspline bspline;
            bspline.order = planner_manager_->pp_.bspline_degree_;
            bspline.start_time = info->start_time_;
            bspline.traj_id = info->traj_id_;
            Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
            for (int i = 0; i < pos_pts.rows(); ++i)
            {
                geometry_msgs::Point pt;
                pt.x = pos_pts(i, 0);
                pt.y = pos_pts(i, 1);
                pt.z = pos_pts(i, 2);
                bspline.pos_pts.push_back(pt);
            }
            Eigen::VectorXd knots = info->position_traj_.getKnot();
            for (int i = 0; i < knots.rows(); ++i)
            {
                bspline.knots.push_back(knots(i));
            }
            Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
            for (int i = 0; i < yaw_pts.rows(); ++i)
            {
                double yaw = yaw_pts(i, 0);
                bspline.yaw_pts.push_back(yaw);
            }
            bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
            fd_->newest_traj_ = bspline;
            visualizeLocalTraj();
        }

        return res;
    }

    void FastExplorationFSM::visualizeOdom()
    {
        std::string ns = "uav" + std::to_string(getId());
        if (fd_->record_count_ == fp_->record_interval_)
        {
            fd_->record_count_ = 0;
            fd_->hstory_path.push_back(fd_->odom_pos_);
            visualization_->drawPath(fd_->hstory_path);
            ros::Time t1 = ros::Time::now();
        }
        vector<Eigen::Vector3d> uav_pos = {fd_->odom_pos_};
        vector<Eigen::Vector3d> arrow_end = {fd_->odom_pos_ + fd_->odom_vel_ * 1};
        // draw velocity direction
        visualization_->drawArrows(uav_pos, arrow_end, Eigen::Vector3d(0.1, 0.2, 0.2), Eigen::Vector4d(1, 0, 1, 1), ns, 1, 2);
        // draw UAV position
        visualization_->drawSpheres(uav_pos, 0.3, Eigen::Vector4d(0, 1, 0, 1), ns, 2, 2);
        fd_->record_count_++;
    }

    void FastExplorationFSM::visualizGlobalPath()
    {
        vector<Vector3d> global_tour;
        vector<double> global_yaw;
        vector<Eigen::Vector3d> list1;
        vector<Eigen::Vector3d> list2;
        std::string ns = "uav" + std::to_string(getId());
        expl_manager_->getVisualizationInfo(global_tour, global_yaw);

        if (global_tour.size() > 0)
        {
            list2.push_back(global_tour[0]);
            list1.push_back(fd_->odom_pos_);
            for (int i = 0; i < global_tour.size(); i++)
            {
                list1.push_back(global_tour[i]);
            }
        }

        // draw TSP path
        visualization_->drawLines(list1, 0.1, Eigen::Vector4d(1, 1, 1, 0.5), ns, 1, 0);
        // draw target position
        visualization_->drawSpheres(list2, 0.3, Eigen::Vector4d(0, 0, 1, 1), ns, 2, 0);
    }

    void FastExplorationFSM::visualizeLocalTraj()
    {
        auto info = &planner_manager_->local_data_;
        std::string ns = "uav" + std::to_string(getId());
        visualization_->drawSpheres(expl_manager_->ed_->path_next_goal_, 0.1, Eigen::Vector4d(1, 1, 1, 1), ns, 1, 1);
        visualization_->drawBspline(info->position_traj_, 0.06, Eigen::Vector4d(1.0, 1.0, 0.0, 1), true, 0.06, Eigen::Vector4d(1, 0, 0, 1), ns, 1, 1);
        visualization_->drawBspline(info->tmp_traj_, 0.06, Eigen::Vector4d(0, 0.5, 1, 1), true, 0.06, Eigen::Vector4d(0, 1, 0, 1), ns, 2, 2);
    }

    void FastExplorationFSM::visualizeFrontier()
    {
        // initialize random number generator
        static std::mt19937_64 engine;
        engine.seed(ros::Time::now().toNSec());
        std::uniform_int_distribution<int> distribution(0, 255);

        // initialize viewpoint and frontier data for visulization
        auto ft = expl_manager_->frontier_finder_;
        vector<Frontier> frontiers;
        ft->mygetFrontiers(frontiers);
        geometry_msgs::PoseArray vp_pose_array;
        vector<Eigen::Vector3d> vp_text_pos_array;
        vector<string> vp_text_content_array;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr ft_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        vector<Eigen::Vector3d> ft_text_pos_array;
        vector<string> ft_text_content_array;
        vector<Eigen::Vector4d> color_array;

        // set data
        for (auto ft : frontiers)
        {
            int j = 0; // frontier voxel iterator
            int h = 0; // frontier viewpoint iterator

            // set color
            int r1 = distribution(engine);
            int r2 = distribution(engine);
            int r3 = distribution(engine);
            int a = 255;
            Eigen::Vector4d color(r1, r2, r3, a);
            color_array.push_back(color / 255);

            // set frontier cloud
            while (j < ft.cells_.size())
            {
                pcl::PointXYZRGBA pt;
                pt.x = ft.cells_[j].x();
                pt.y = ft.cells_[j].y();
                pt.z = ft.cells_[j].z();
                pt.r = r1;
                pt.g = r2;
                pt.b = r3;
                // pt.a = 50;
                ft_cloud->push_back(pt);
                j++;
            }

            // set frontier text positon
            Eigen::Vector3d ft_text_pos = ft.average_;
            ft_text_pos.x() += 0.5;
            ft_text_pos.y() += 0.5;
            ft_text_pos_array.push_back(ft_text_pos);

            // set frontier text content
            string ft_text_content = "FRT " + std::to_string(ft.id_) + " CELL " + std::to_string(ft.cells_.size()) + " TYPE " + std::to_string(ft.type_);
            ft_text_content_array.push_back(ft_text_content);

            // set viewpoints visualization data
            // while(h<ft.viewpoints_.size())
            while (h < 1) // only show best viewpoint
            {
                // set viewpoint pose
                geometry_msgs::Pose pose;
                pose.position.x = ft.viewpoints_[h].pos_(0);
                pose.position.y = ft.viewpoints_[h].pos_(1);
                pose.position.z = ft.viewpoints_[h].pos_(2);

                Eigen::Matrix3d R_wb;
                R_wb << cos(ft.viewpoints_[h].yaw_), -sin(ft.viewpoints_[h].yaw_), 0.0, sin(ft.viewpoints_[h].yaw_), cos(ft.viewpoints_[h].yaw_), 0.0, 0.0, 0.0, 1.0;
                Eigen::Quaterniond q(R_wb);
                pose.orientation.w = q.w();
                pose.orientation.x = q.x();
                pose.orientation.y = q.y();
                pose.orientation.z = q.z();
                vp_pose_array.poses.push_back(pose);

                // set viewpoint text popsition
                Eigen::Vector3d vp_text_pos;
                vp_text_pos.x() = ft.viewpoints_[h].pos_(0);
                vp_text_pos.y() = ft.viewpoints_[h].pos_(1);
                vp_text_pos.z() = ft.viewpoints_[h].pos_(2);
                vp_text_pos_array.push_back(vp_text_pos);

                // set viewpoint text content
                string vp_text_content = "VP " + std::to_string(ft.id_) + "visib_num" + std::to_string(ft.viewpoints_[h].visib_num_) + "qua" + std::to_string(ft.viewpoints_[h].quality_);
                vp_text_content_array.push_back(vp_text_content);
                h++;
            }

            // draw target frontier normal
            if (ft.is_target_)
            {
                vector<Eigen::Vector3d> start_list;
                vector<Eigen::Vector3d> end_list;
                start_list.push_back(ft.average_);
                end_list.push_back(ft.average_ + ft.normal_);
                std::string ns = "uav" + std::to_string(getId());
                visualization_->drawArrows(start_list, end_list, Eigen::Vector3d(0.1, 0.2, 0.2), Eigen::Vector4d(1, 1, 1, 1), ns, 1, 3);
            }
        }
        // visualization
        std::string ns = "uav" + std::to_string(getId());
        visualization_->drawCloud(ft_cloud);
        visualization_->drawPoseArray(vp_pose_array);
        visualization_->drawTextArray(ft_text_pos_array, ft_text_content_array, marker_scale_, color_array, ns, 1);
        visualization_->drawTextArray(vp_text_pos_array, vp_text_content_array, marker_scale_, color_array, ns, 2);
    }

    void FastExplorationFSM::visualizeEnvironment()
    {
        // get origin and size
        Eigen::Vector3d map_origin, box_origin;
        Eigen::Vector3d map_size, box_size;
        Eigen::Vector3d box_min;
        Eigen::Vector3d box_max;
        planner_manager_->edt_environment_->sdf_map_->getRegion(map_origin, map_size);
        planner_manager_->edt_environment_->sdf_map_->getBox(box_min, box_max);
        box_origin = box_min;
        box_size = box_max - box_min;

        // calculate edge line list
        vector<Eigen::Vector3d> list_map_1, list_map_2;
        vector<Eigen::Vector3d> list_box_1, list_box_2;
        visualization_->getCuboidEdge(list_map_1, list_map_2, map_origin, map_size);
        visualization_->getCuboidEdge(list_box_1, list_box_2, box_origin, box_size);
        visualization_->drawLines(list_map_1, list_map_2, 0.1, Eigen::Vector4d(1, 0.5, 0, 1), "", 1, 6);
        visualization_->drawLines(list_box_1, list_box_2, 0.1, Eigen::Vector4d(1, 1, 1, 1), "", 2, 6);
    }

    void FastExplorationFSM::visualizeUpdateBox()
    {
        Vector3d update_min, update_max;
        planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(update_min, update_max, true);
        vector<Eigen::Vector3d> list_box_1, list_box_2;
        visualization_->getCuboidEdge(list_box_1, list_box_2, update_min, update_max - update_min);
        visualization_->drawLines(list_box_1, list_box_2, 0.1, Eigen::Vector4d(0, 0.2, 1, 0.8), "", 3, 6);
    }

}
