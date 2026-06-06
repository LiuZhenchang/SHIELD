#ifndef _EXPLORATION_MANAGER_H_
#define _EXPLORATION_MANAGER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>

using Eigen::Vector3d;
using std::pair;
using std::shared_ptr;
using std::tuple;
using std::unique_ptr;
using std::vector;

namespace shield
{
    class EDTEnvironment;
    class SDFMap;
    class FastPlannerManager;
    class HGrid;
    struct ExplorationParam;
    struct ExplorationData;

    class FrontierFinder;

    enum EXPL_RESULT { NO_FRONTIER, FAIL, SUCCEED };

    class FastExplorationManager
    {
    public:
        FastExplorationManager();
        ~FastExplorationManager();

        void initialize(ros::NodeHandle &nh);

        // Find optimal tour visiting unknown grid
        bool findGlobalTourOfGrid(
            const vector<Eigen::Vector3d> &positions,
            const vector<Eigen::Vector3d> &velocities,
            vector<int> &ids,
            bool init = false);

        void findGridAndFrontierPath(const Vector3d &cur_pos,
                                     const Vector3d &cur_vel, const double &cur_yaw, vector<int> &grid_ids,
                                     vector<int> &frontier_ids);
        void findTourOfFrontier(const Vector3d &cur_pos, const Vector3d &cur_vel,
                                const double &cur_yaw, const vector<int> &ftr_ids, const vector<Eigen::Vector3d> &grid_pos,
                                vector<int> &indices);

        int planTrajToView( const Vector3d& pos, 
                            const Vector3d& vel, 
                            const Vector3d& acc,
                            const Vector3d& yaw, 
                            const Vector3d& next_pos, 
                            const double& next_yaw);


        shared_ptr<ExplorationData> ed_;
        shared_ptr<ExplorationParam> ep_;
        shared_ptr<FastPlannerManager> planner_manager_;
        shared_ptr<HGrid> hgrid_;
        shared_ptr<SDFMap> sdf_map_;

        //flt added frontier

        shared_ptr<FrontierFinder> frontier_finder_;

        int planExploreMotion(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw);
        void findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw, vector<int>& indices);
        void shortenPath(vector<Vector3d>& path);
        void getVisualizationInfo( vector<Vector3d>& global_tour, vector<double>& global_yaw);

    private:
        shared_ptr<EDTEnvironment> edt_environment_;
        ros::ServiceClient tsp_client_;
    };
}

#endif
