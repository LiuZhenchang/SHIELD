#include <exploration_manager/shield_exploration_manager.h>
#include <exploration_manager/expl_data.h>
#include <active_perception/frontier_finder.h>
#include <active_perception/hgrid.h>
#include <active_perception/graph_node.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <plan_manage/planner_manager.h>

/******************LZC added program******************/
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
/*****************************************************/

#include <lkh_tsp_solver/lkh_interface.h>
#include <lkh_mtsp_solver/SolveMTSP.h>

#include <thread>
#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;

namespace shield
{
    FastExplorationManager::FastExplorationManager()
    {
    }

    FastExplorationManager::~FastExplorationManager()
    {
        ViewNode::astar_.reset();
        ViewNode::caster_.reset();
        ViewNode::map_.reset();
    }

    void FastExplorationManager::initialize(ros::NodeHandle &nh)
    {
        planner_manager_.reset(new FastPlannerManager);
        planner_manager_->initPlanModules(nh);

        edt_environment_ = planner_manager_->edt_environment_;
        sdf_map_ = edt_environment_->sdf_map_;
        frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));

        hgrid_.reset(new HGrid(edt_environment_, nh));

        ed_.reset(new ExplorationData);
        ep_.reset(new ExplorationParam);

        nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
        nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
        nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
        nh.param("exploration/drone_num", ep_->drone_num_, 1);
        nh.param("exploration/drone_id", ep_->drone_id_, 1);
        nh.param("exploration/init_plan_num", ep_->init_plan_num_, 2);
        nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
        nh.param("exploration/mtsp_dir", ep_->mtsp_dir_, string("null"));
        nh.param("exploration/traj_segment_length", ep_->traj_segment_length_, 7.0);


        ed_->swarm_state_.resize(ep_->drone_num_);
        ed_->pair_opt_stamps_.resize(ep_->drone_num_);
        ed_->pair_opt_res_stamps_.resize(ep_->drone_num_);
        for (int i = 0; i < ep_->drone_num_; ++i)
        {
            ed_->swarm_state_[i].stamp_ = 0.0;
            ed_->pair_opt_stamps_[i] = 0.0;
            ed_->pair_opt_res_stamps_[i] = 0.0;
        }
        
        int hgrid_num = hgrid_->getHGridNum();

        for (int i = 0; i < hgrid_num; ++i)
        {
            ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_.push_back(i);
        }

        nh.param("exploration/vm", ViewNode::vm_, -1.0);
        nh.param("exploration/am", ViewNode::am_, -1.0);
        nh.param("exploration/yd", ViewNode::yd_, -1.0);
        nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
        nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

        ViewNode::astar_.reset(new Astar);
        ViewNode::astar_->init(nh, edt_environment_);
        ViewNode::map_ = sdf_map_;

        double resolution_ = sdf_map_->getResolution();
        Eigen::Vector3d origin, size;
        sdf_map_->getRegion(origin, size);
        ViewNode::caster_.reset(new RayCaster);
        ViewNode::caster_->setParams(resolution_, origin);

        // Swarm
        for (auto &state : ed_->swarm_state_)
        {
            state.stamp_ = 0.0;
            state.ego_ids_ = {};
            state.explored_ids_ = {};
        }
        // ed_->last_grid_ids_ = {};
        ed_->reallocated_ = true;
        ed_->wait_response_ = false;
        ed_->current_explore_grid_id_ = -1;
        ed_->plan_num_ = 0;
        ed_->explore_finish_ = false;

        // Initialize TSP par file
        ofstream par_file(ep_->tsp_dir_ + "/single.par");
        par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
        par_file << "GAIN23 = NO\n";
        par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
        par_file << "RUNS = 1\n";

        ofstream par_file_grid(ep_->tsp_dir_ + "/single_grid.par");
        par_file_grid << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single_grid.tsp\n";
        par_file_grid << "GAIN23 = NO\n";
        par_file_grid << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single_grid.txt\n";
        par_file_grid << "RUNS = 1\n";

        //cp更改：以service形式调用tsp
        tsp_client_ =
            nh.serviceClient<lkh_mtsp_solver::SolveMTSP>("/solve_tsp_" + to_string(ep_->drone_id_), true);
        
    }

    int FastExplorationManager::planTrajToView(const Vector3d &pos,
                                               const Vector3d &vel,
                                               const Vector3d &acc,
                                               const Vector3d &yaw,
                                               const Vector3d &next_pos,
                                               const double &next_yaw)
    {
        // Plan trajectory (position and yaw) to the next viewpoint
        auto t1 = ros::Time::now();

        // Compute time lower bound of yaw and use in trajectory generation
        double end_yaw = next_yaw;
        double diff0 = next_yaw - yaw[0];
        double diff1 = fabs(diff0);
        double time_lb = min(diff1, 2 * M_PI - diff1) / ViewNode::yd_;

        // Generate trajectory of x,y,z
        bool optimistic = ed_->plan_num_ < ep_->init_plan_num_;
        planner_manager_->path_finder_->reset();
        if (planner_manager_->path_finder_->search(pos, next_pos, optimistic) != Astar::REACH_END)
        {
            ROS_ERROR("No path to next viewpoint");
            return FAIL;
        }
        ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
        shortenPath(ed_->path_next_goal_);

        if (ed_->path_next_goal_.size() < 2)
        {
            ROS_ERROR("Path size less than 2");
            return FAIL;
        }

        ed_->kino_path_.clear();

        double radius_far = ep_->traj_segment_length_;
        const double len = Astar::pathLength(ed_->path_next_goal_);

        // Increment the planning number if in the initial planning phase
        if (ed_->plan_num_ < ep_->init_plan_num_)
        {
            ed_->plan_num_++;
            ROS_WARN("init plan.");
        }
        // Determine the planning strategy based on the path length
        if (len > radius_far)
        {
            std::cout << "\033[42;37m Far goal. \033[0m" << std::endl;
            double len2 = 0.0;
            vector<Eigen::Vector3d> truncated_path = {ed_->path_next_goal_.front()};
            for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i)
            {
                auto cur_pt = ed_->path_next_goal_[i];
                len2 += (cur_pt - truncated_path.back()).norm();
                truncated_path.push_back(cur_pt);
            }
            ed_->next_goal_ = truncated_path.back();
            Eigen::Vector3d goal2nextpos = next_pos - ed_->next_goal_;
            if ((len - radius_far) > 0.5)
            {
                end_yaw = atan2(goal2nextpos(1), goal2nextpos(0));
            }
            // Adjust time lower bound of yaw and use in trajectory generation
            diff0 = end_yaw - yaw[0];
            diff1 = fabs(diff0);
            while (diff1 > 2 * M_PI)
            {
                diff1 -= 2 * M_PI;
            }
            time_lb = min(diff1, 2 * M_PI - diff1) / ViewNode::yd_;
            planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
        }
        else
        {
            ed_->next_goal_ = next_pos;
            planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
        }

        // Check if the trajectory planning time meets the time lower bound
        if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.5)
            ROS_ERROR("Lower bound not satified!");

        // Calculate the time taken for trajectory planning
        double traj_plan_time = (ros::Time::now() - t1).toSec();

        // plan yaw trajectory
        t1 = ros::Time::now();
        planner_manager_->planYawExplore(yaw, end_yaw, true, ep_->relax_time_);
        double yaw_time = (ros::Time::now() - t1).toSec();
        ROS_INFO("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);

        bool check_pos = planner_manager_->checkPositionTraj();
        planner_manager_->checkYawTraj();
        if (check_pos)
            return SUCCEED;
        else
            return FAIL;
    }

    int FastExplorationManager::planExploreMotion(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc, const Vector3d &yaw)
    {
        ROS_INFO_STREAM("\033[42;37m planExploreMotion start \033[0m");
        ros::Time t1 = ros::Time::now();
        frontier_finder_->searchFrontiers(pos, yaw[0]);
        frontier_finder_->computeFrontiersToVisit(pos);

        ed_->views_.clear();
        ed_->global_tour_.clear();
        ed_->global_yaw_.clear();

        std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
                  << ", acc: " << acc.transpose() << std::endl;

        frontier_finder_->getFrontiers(ed_->frontiers_);

        if (ed_->frontiers_.empty())
        {
            ROS_WARN("planExploreMotion: No coverable frontier.");
            ROS_INFO_STREAM("\033[42;37m planExploreMotion end - NO_FRONTIER \033[0m");
            return NO_FRONTIER;
        }
        frontier_finder_->getTopViewpointsInfo(pos, ed_->views_, ed_->yaws_, ed_->averages_);

        double view_time = (ros::Time::now() - t1).toSec();
        ROS_WARN("Frontier: %d, viewpoint: %ld, t: %lf", ed_->frontiers_.size(), ed_->views_.size(), view_time);

        // // Do global and local tour planning and retrieve the next viewpoint
        // Vector3d next_pos;
        // double next_yaw;
        // Reset the target frontier
        int target_frontier_id = -1;
        if (ed_->views_.size() > 1)
        {
            // Find the global tour passing through all viewpoints
            // Create TSP and solve by LKH
            // Optimal tour is returned as indices of frontier
            vector<int> indices;
            findGlobalTour(pos, vel, yaw, indices);
            target_frontier_id = indices[0];
        }
        else if (ed_->views_.size() == 1)
        {
            // Only 1 destination, no need to find global tour through TSP
            frontier_finder_->updateFrontierCostMatrix();
            ed_->global_tour_ = {ed_->views_[0]};
            ed_->global_yaw_ = {ed_->yaws_[0]};
            target_frontier_id = 0;
        }
        else
        {
            ROS_ERROR("planExploreMotion: Empty destination.");
            ROS_INFO_STREAM("\033[42;37m planExploreMotion end - FAIL \033[0m");
            return FAIL;
        }
        if (!frontier_finder_->haveTargetFrontier())
        {
            // frontier_finder_->resetTargetFrontier();
            frontier_finder_->setTargetFrontier(target_frontier_id);
            ed_->next_pos_ = ed_->global_tour_[0];
            ed_->next_yaw_ = ed_->global_yaw_[0];
        }
        else
        {
            ROS_WARN("planExploreMotion: Continue to explore the last frontier.");
        }
        ROS_INFO_STREAM("\033[42;37m planExploreMotion end - SUCCEED \033[0m");
        return SUCCEED;
    }

    void FastExplorationManager::shortenPath(vector<Vector3d> &path)
    {
        if (path.empty())
        {
            ROS_ERROR("Empty path to shorten");
            return;
        }
        if (path.size() < 5)
            return;
        // Shorten the tour, only critical intermediate points are reserved.
        const double dist_thresh = 1.0;
        vector<Vector3d> short_tour = {path.front()};
        for (int i = 1; i < path.size() - 1; ++i)
        {
            if ((path[i] - short_tour.back()).norm() > dist_thresh)
                short_tour.push_back(path[i]);
            else
            {
                // Add waypoints to shorten path only to avoid collision
                ViewNode::caster_->input(short_tour.back(), path[i + 1]);
                Eigen::Vector3i idx;
                while (ViewNode::caster_->nextId(idx) && ros::ok())
                {
                    if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
                        edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN)
                    {
                        short_tour.push_back(path[i]);
                        break;
                    }
                }
            }
        }
        if ((path.back() - short_tour.back()).norm() > 1e-3)
            short_tour.push_back(path.back());

        // Ensure at least three points in the path
        if (short_tour.size() == 2)
            short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
        path = short_tour;
    }

    //cp更改：计算hgrid的路径
    bool FastExplorationManager::findGlobalTourOfGrid(
        const vector<Eigen::Vector3d> &positions,
        const vector<Eigen::Vector3d> &velocities,
        vector<int> &indices,
        bool init)
    {
        if (ed_->explore_finish_)
        {
            return false;
        }

        struct timeval t1, t2;
        gettimeofday(&t1, NULL);

        Eigen::MatrixXd cost_mat;

        std::cout << "positions=" << positions[0] << std::endl;
        auto &grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;

        if (grid_ids.empty())
        {
            ROS_WARN("UAV %d explore Finished 1.", ep_->drone_id_);
            // ed_->explore_finish_ = true; flt先注释掉，避免影响状态机
            return false;

        }
        else
        {
            vector<Frontier> frt;
            frontier_finder_->mygetFrontiers(frt);
            // hgrid_->inputFrontiers(ed_->averages_);
            hgrid_->inputFrontiers(frt);

            hgrid_->updateGridData(grid_ids);
            vector<int> active_ids;
            hgrid_->getActiveGrids(active_ids);
            if(active_ids.size()<=1){
                indices = active_ids;
            }
            else{

                struct timeval t3, t4;
                gettimeofday(&t3, NULL);
                hgrid_->getCostMatrix(positions[0],velocities[0], active_ids, cost_mat);

                gettimeofday(&t4, NULL);
                long seconds = t4.tv_sec - t3.tv_sec;
                long usec = t4.tv_usec - t3.tv_usec;
                double elapsed2 = seconds + usec / 1e6;
                ROS_INFO_STREAM("\033[45;37m Hgrid cost matrix Elapsed time: "<< elapsed2 <<" seconds "<<"\033[0m");


                const int dimension = cost_mat.rows();

                // Write params and cost matrix to problem file
                ofstream prob_file(ep_->tsp_dir_ + "/single_grid.tsp");
                // Problem specification part, follow the format of TSPLIB

                string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
                                "\nEDGE_WEIGHT_TYPE : "
                                "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

                // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " + to_string(dimension) +
                //     "\nEDGE_WEIGHT_TYPE : "
                //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

                prob_file << prob_spec;
                // prob_file << "TYPE : TSP\n";
                // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
                // Problem data part
                const int scale = 100;
                if (false)
                {
                    // Use symmetric TSP
                    for (int i = 1; i < dimension; ++i)
                    {
                        for (int j = 0; j < i; ++j)
                        {
                            int int_cost = cost_mat(i, j) * scale;
                            prob_file << int_cost << " ";
                        }
                        prob_file << "\n";
                    }
                }
                else
                {
                    // Use Asymmetric TSP
                    for (int i = 0; i < dimension; ++i)
                    {
                        for (int j = 0; j < dimension; ++j)
                        {
                            int int_cost = cost_mat(i, j) * scale;
                            prob_file << int_cost << " ";
                        }
                        prob_file << "\n";
                    }
                }

                prob_file << "EOF";
                prob_file.close();

                // Call LKH TSP solver
                int result = solveTSPLKH((ep_->tsp_dir_ + "/single_grid.par").c_str());

                // Read optimal tour from the tour section of result file
                ifstream res_file(ep_->tsp_dir_ + "/single_grid.txt");
                string res;
                while (getline(res_file, res))
                {
                    // Go to tour section
                    if (res.compare("TOUR_SECTION") == 0)
                        break;
                }

                if (false)
                {
                    // Read path for Symmetric TSP formulation
                    getline(res_file, res); // Skip current pose
                    getline(res_file, res);
                    int id = stoi(res);
                    bool rev = (id == dimension); // The next node is virutal depot?

                    while (id != -1)
                    {
                        indices.push_back(active_ids[id - 2]);
                        getline(res_file, res);
                        id = stoi(res);
                    }
                    if (rev)
                        reverse(indices.begin(), indices.end());
                    indices.pop_back(); // Remove the depot
                }
                else
                {
                    // Read path for ATSP formulation
                    while (getline(res_file, res))
                    {
                        // Read indices of frontiers in optimal tour
                        int id = stoi(res);
                        if (id == 1) // Ignore the current state
                            continue;
                        if (id == -1)
                            break;
                        indices.push_back(active_ids[id - 2]); // Idx of solver-2 == Idx of frontier
                    }
                }

                res_file.close();
            }
        }


        
        // if(ed_->current_explore_grid_id_ == -1){
        //     ed_->current_explore_grid_id_ = indices.front();
        // }
        // bool found_current = false;
        // for(int i=0;i<indices.size();i++){
        //     if(indices[i] == ed_->current_explore_grid_id_){
        //         indices.erase(indices.begin()+i);
        //         indices.insert(indices.begin(), ed_->current_explore_grid_id_);
        //         found_current = true;
        //         break;
        //     }
        // }
        // if(!found_current){
        //     ed_->current_explore_grid_id_ = indices.front();
        // }

        grid_ids = indices;
        hgrid_->getGridTour(grid_ids, positions[0], ed_->grid_tour_);

        gettimeofday(&t2, NULL);
        long seconds = t2.tv_sec - t1.tv_sec;
        long usec = t2.tv_usec - t1.tv_usec;
        double elapsed = seconds + usec / 1e6;

        return true;
    }
    //cp更改：负责挑出来第一个hgrid里面的frontier，和下一个hgrid
    void FastExplorationManager::findGridAndFrontierPath(const Vector3d &cur_pos,
                                                         const Vector3d &cur_vel, const double &cur_yaw, vector<int> &grid_ids,
                                                         vector<int> &frontier_ids)
    {

        struct timeval t1, t2;
        gettimeofday(&t1, NULL);


        if (grid_ids.empty())
        {
            return;
        }
        vector<int> frt_ids;
        hgrid_->getFrontiersInGrid(grid_ids, frt_ids);
        ROS_INFO("Find frontier tour, %d involved------------", frt_ids.size());

        if (frt_ids.empty())
        {
            frontier_ids = {};
            return;
        }

        std::cout << "First Grid id = " << grid_ids.front();
        for (int i = 0; i < frt_ids.size(); i++)
        {
            std::cout << " Frt id = " << frt_ids[i];
        }
        std::cout << std::endl;

        // Consider next grid in frontier tour planning
        Eigen::Vector3d grid_pos;
        vector<Eigen::Vector3d> grid_pos_vec;
        // 写入对应grid_ids的unknown center到grid_pos_vec中
        for (int i = 0; i < grid_ids.size(); ++i)
        {
            grid_pos_vec.push_back(hgrid_->getCenter(grid_ids[i], 2));
        }


        if(grid_pos_vec.size()>=2){
            Eigen::Vector3d next_grid_pos = grid_pos_vec[1]; // 读取下一个grid的位置
            findTourOfFrontier(cur_pos, cur_vel, cur_yaw, frt_ids, {next_grid_pos}, frontier_ids);
        }
        else if(grid_pos_vec.size()==1){
            ROS_INFO_STREAM("\033[43;37m Left hgrid size = 1 \033[0m");
            findTourOfFrontier(cur_pos, cur_vel, cur_yaw, frt_ids, {}, frontier_ids);
            ROS_INFO_STREAM("\033[43;37m Left hgrid size = 1 end \033[0m");

        }
        else{

        }
        gettimeofday(&t2, NULL);
        long seconds = t2.tv_sec - t1.tv_sec;
        long usec = t2.tv_usec - t1.tv_usec;
        double elapsed = seconds + usec / 1e6;



    }
    //cp更改：被findGridAndFrontierPath调用，对第一个hgrid里面的frontier，和下一个hgrid进行tsp计算
    void FastExplorationManager::findTourOfFrontier(const Vector3d &cur_pos, const Vector3d &cur_vel,
                                                    const double &cur_yaw, const vector<int> &ftr_ids, const vector<Eigen::Vector3d> &grid_pos,
                                                    vector<int> &indices)
    {

        auto t1 = ros::Time::now();

        vector<Eigen::Vector3d> positions = {cur_pos};
        vector<Eigen::Vector3d> velocities = {cur_vel};
        vector<double> yaws = {cur_yaw};

        Eigen::MatrixXd mat;
        frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, ftr_ids, grid_pos, mat);
        const int dimension = mat.rows();



        double mat_time = (ros::Time::now() - t1).toSec();

        // Find optimal allocation through AmTSP
        t1 = ros::Time::now();

        // Create problem file
        ofstream file(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp");
        file << "NAME : amtsp\n";
        file << "TYPE : ATSP\n";
        file << "DIMENSION : " + to_string(dimension) + "\n";
        file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
        file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
        file << "EDGE_WEIGHT_SECTION\n";
        for (int i = 0; i < dimension; ++i)
        {
            for (int j = 0; j < dimension; ++j)
            {
                int int_cost = 100 * mat(i, j);
                file << int_cost << " ";
            }
            file << "\n";
        }
        file.close();

        // Create par file
        const int drone_num = 1;

        file.open(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".par");
        file << "SPECIAL\n";
        file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp\n";
        file << "SALESMEN = " << to_string(drone_num) << "\n";
        file << "MTSP_OBJECTIVE = MINSUM\n";
        file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) << "\n";
        file << "MTSP_MAX_SIZE = "
             << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
        file << "RUNS = 1\n";
        file << "TRACE_LEVEL = 0\n";
        file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour\n";
        file.close();

        auto par_dir = ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp";
        t1 = ros::Time::now();

        lkh_mtsp_solver::SolveMTSP srv;
        srv.request.prob = 1;
        if (!tsp_client_.call(srv))
        {
            ROS_ERROR("Fail to solve ATSP.");
            return;
        }

        // int result = solveTSPLKH(par_dir.c_str());

        // if(result != 0)
        // {
        //     return;
        // }

        double mtsp_time = (ros::Time::now() - t1).toNSec();

        // Read results
        t1 = ros::Time::now();

        ifstream fin(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour");
        string res;
        vector<int> ids;
        while (getline(fin, res))
        {
            if (res.compare("TOUR_SECTION") == 0)
                break;
        }
        while (getline(fin, res))
        {
            int id = stoi(res);
            ids.push_back(id - 1);
            if (id == -1)
                break;
        }
        fin.close();

        // Parse the m-tour
        vector<vector<int>> tours;
        vector<int> tour;
        for (auto id : ids)
        {
            if (id > 0 && id <= drone_num)
            {
                tour.clear();
                tour.push_back(id);
            }
            else if (id >= dimension || id <= 0)
            {
                tours.push_back(tour);
            }
            else
            {
                tour.push_back(id);
            }
        }

        vector<vector<int>> others(drone_num - 1);
        for (int i = 1; i < tours.size(); ++i)
        {
            if (tours[i][0] == 1)
            {
                indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
            }
            // else {
            //   others[tours[i][0] - 2].insert(
            //       others[tours[i][0] - 2].end(), tours[i].begin() + 1, tours[i].end());
            // }
        }
        for (auto &id : indices)
        {
            id -= 1 + drone_num;
        }
        // for (auto& other : others) {
        //   for (auto& id : other)
        //     id -= 1 + drone_num;
        // }

        if (ed_->grid_tour_.size() > 2)
        { // Remove id for next grid, since it is considered in the TSP
            indices.pop_back();
        }
        // Subset of frontier inside first grid
        for (int i = 0; i < indices.size(); ++i)
        {
            indices[i] = ftr_ids[indices[i]];
        }

        // Get the path of optimal tour from path matrix
        frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);
        if (!grid_pos.empty())
        {
            ed_->frontier_tour_.push_back(grid_pos[0]);
        }

        // ed_->other_tours_.clear();
        // for (int i = 1; i < positions.size(); ++i) {
        //   ed_->other_tours_.push_back({});
        //   frontier_finder_->getPathForTour(positions[i], others[i - 1], ed_->other_tours_[i - 1]);
        // }

        double parse_time = (ros::Time::now() - t1).toSec();
        //     parse_time, indices.size());
    }

    void FastExplorationManager::findGlobalTour(
        const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw,
        vector<int> &indices)
    {
        auto t1 = ros::Time::now();

        // Get cost matrix for current state and clusters
        Eigen::MatrixXd cost_mat;
        frontier_finder_->updateFrontierCostMatrix();
        frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
        const int dimension = cost_mat.rows();

        double mat_time = (ros::Time::now() - t1).toNSec();
        t1 = ros::Time::now();

        // Write params and cost matrix to problem file
        ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
        // Problem specification part, follow the format of TSPLIB

        string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
                           "\nEDGE_WEIGHT_TYPE : "
                           "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

        // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " + to_string(dimension) +
        //     "\nEDGE_WEIGHT_TYPE : "
        //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

        prob_file << prob_spec;
        // prob_file << "TYPE : TSP\n";
        // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
        // Problem data part
        const int scale = 100;
        if (false)
        {
            // Use symmetric TSP
            for (int i = 1; i < dimension; ++i)
            {
                for (int j = 0; j < i; ++j)
                {
                    int int_cost = cost_mat(i, j) * scale;
                    prob_file << int_cost << " ";
                }
                prob_file << "\n";
            }
        }
        else
        {
            // Use Asymmetric TSP
            for (int i = 0; i < dimension; ++i)
            {
                for (int j = 0; j < dimension; ++j)
                {
                    int int_cost = cost_mat(i, j) * scale;
                    prob_file << int_cost << " ";
                }
                prob_file << "\n";
            }
        }

        prob_file << "EOF";
        prob_file.close();

        // Call LKH TSP solver
        int result = solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

        // Read optimal tour from the tour section of result file
        ifstream res_file(ep_->tsp_dir_ + "/single.txt");
        string res;
        while (getline(res_file, res))
        {
            // Go to tour section
            if (res.compare("TOUR_SECTION") == 0)
                break;
        }

        if (false)
        {
            // Read path for Symmetric TSP formulation
            getline(res_file, res); // Skip current pose
            getline(res_file, res);
            int id = stoi(res);
            bool rev = (id == dimension); // The next node is virutal depot?

            while (id != -1)
            {
                indices.push_back(id - 2);
                getline(res_file, res);
                id = stoi(res);
            }
            if (rev)
                reverse(indices.begin(), indices.end());
            indices.pop_back(); // Remove the depot
        }
        else
        {
            // Read path for ATSP formulation
            while (getline(res_file, res))
            {
                // Read indices of frontiers in optimal tour
                int id = stoi(res);
                if (id == 1) // Ignore the current state
                    continue;
                if (id == -1)
                    break;
                indices.push_back(id - 2); // Idx of solver-2 == Idx of frontier
            }
        }

        res_file.close();

        // Get the path of optimal tour from path matrix
        // frontier_finder_->getPathForTour(cur_pos, indices, ed_->global_tour_);

        for (int i = 0; i < indices.size(); i++)
        {
            ed_->global_tour_.push_back(ed_->views_[indices[i]]);
            ed_->global_yaw_.push_back(ed_->yaws_[indices[i]]);
        }

        double tsp_time = (ros::Time::now() - t1).toNSec();
        ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
    }

    void FastExplorationManager::getVisualizationInfo( vector<Vector3d>& global_tour, vector<double>& global_yaw)
    {
        global_tour = ed_->global_tour_;
        global_yaw = ed_->global_yaw_;
    }

}
