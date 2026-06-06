#ifndef _HGRID_H_
#define _HGRID_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <unordered_map>
#include <utility>
#include <algorithm> 
#include <active_perception/frontier_finder.h>


using Eigen::Vector3d;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

namespace shield
{
    class EDTEnvironment;
    class Astar;
    class GridInfo;
    class UniformGrid;
    // Hierarchical grid, contains two levels currently
    class HGrid
    {
    public:
        HGrid(const shared_ptr<EDTEnvironment> &edt, ros::NodeHandle &nh);
        ~HGrid();
        void updateGridData(vector<int> &grid_ids);
        void getActiveGrids(vector<int> &grid_ids);
        Eigen::Vector3d getResolution();
        Eigen::Vector3d getCenter(const int &grid_id, int type = 0);
        void getCenterAll(vector<Eigen::Vector3d> &centers, vector<int> &ids, int type = 0);
        double getCostDroneToGrid(const Eigen::Vector3d& pos, const int& grid_id);
        double getCostDroneToGrid(const Eigen::Vector3d &pos, const Eigen::Vector3d &Vel, const int &grid_id);
        double getCostGridToGrid(const int& id1, const int& id2);
        void getCostMatrix(const Eigen::Vector3d& position, const vector<int>& grid_ids, Eigen::MatrixXd& mat);
        void getCostMatrix(const Eigen::Vector3d &position, const Eigen::Vector3d &Vel, const vector<int> &grid_ids, Eigen::MatrixXd &mat);
        double get_unknown_percent(const int& id);
        void getFrontiersInGrid(const vector<int>& grid_ids, vector<int>& frt_ids);
        void inputFrontiers(const vector<Eigen::Vector3d>& avgs);
        void inputFrontiers(const vector<Frontier>& frts);
        void getGridTour(const vector<int> &ids, const Eigen::Vector3d &pos, vector<Eigen::Vector3d> &tour);
        int getHGridNum(void);
        int getFrontierNum(const int &id);
        int getUnknownFrontierNum(const int &id);
        int getQualityFrontierNum(const int &id);
        void deActivateGrid(int &id);
        void dormantGrid(int &id);
        void setDronePos(const Eigen::Vector3d pos);
        int getDroneGridID();


        int getHGridStatus(const int &grid_id);
        int getGridIDofPos(const Eigen::Vector3d &pos);


        unique_ptr<Astar> path_finder_;

    private:
        unique_ptr<UniformGrid> grid_;
        shared_ptr<EDTEnvironment> edt_;
        GridInfo& getGrid(const int& id);

        bool isClose(const int& id1, const int& id2);
        bool inSameLevel1(const int& id1, const int& id2);
        double unknown_percent_thresh_ = 0.05;
        double coefficient_vel_cost_;
        double coefficient_frt_cost_;

        Eigen::Vector3d drone_pos_;
    };

}

#endif