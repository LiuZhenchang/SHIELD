#ifndef _UNIFORM_GRID_H_
#define _UNIFORM_GRID_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <unordered_map>
#include <utility>

#include <active_perception/frontier_finder.h>


using Eigen::Vector3d;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

class RayCaster;

namespace shield
{

  class EDTEnvironment;
  class HGrid;

  // struct GridInfo {};

  class GridInfo
  {
  public:
    GridInfo()
    {
    }
    ~GridInfo()
    {
    }

    int unknown_num_;
    int free_num_;
    int frontier_num_, frontier_quality_num_, frontier_unknown_num_;
    Eigen::Vector3d center_;
    Eigen::Vector3d center_unknown_;
    Eigen::Vector3d center_free_;
    unordered_map<int, int> frontier_cell_nums_;
    unordered_map<int, int> contained_frontier_ids_;
    bool is_updated_;
    bool need_divide_, active_;
    int type_; // 0: active, 1: dormant, 2: deactive

    bool is_prev_relevant_;
    bool is_cur_relevant_;

    // Vertices and their box in xy plane, in current drone's frame
    Eigen::Vector3d vmin_, vmax_;
    vector<Eigen::Vector3d> vertices_;

    // Normals of separating lines in xy plane, associated with vertices_
    vector<Eigen::Vector3d> normals_;

    /************************LZC Add************************/
    enum
    {
      HAVE_PATH = 1,
      NO_PATH = 2,
      UNPLAN = 3
    };
    vector<Eigen::Vector3d> path_node_;
    double path_length_ = -1;
    double uav_distance_ = -1;
    int path_status_;
    int num_execution_ = 0;
    int num_path_fail_ = 0;
    int voxel_num_=0;
    /*******************************************************/
  };

  class UniformGrid
  {

  public:
    UniformGrid(const shared_ptr<EDTEnvironment> &edt, ros::NodeHandle &nh, const int &level);
    ~UniformGrid();

    void initGridData();
    void updateBaseCoor();
    void updateGridData(vector<int> &grid_ids);
    void activateGrid(int &id);
    void activateGrids(const vector<int> &ids);
    void deactivateGrids(const vector<int> &ids);
    void deactivateGrid(int &id);
    void dormantGrid(int &id);


    void inputFrontiers(const vector<Eigen::Vector3d> &avgs);
    void inputFrontiers(const vector<Frontier> &frts);
    void getGridTour(const vector<int> &ids, vector<Eigen::Vector3d> &tour);
    void getFrontiersInGrid(const int &grid_id, vector<int> &ftr_ids);
    void getGridMarker(vector<Eigen::Vector3d> &pts1, vector<Eigen::Vector3d> &pts2);
    int getGridtype(const int &id);


    /********************* LZC Added **********************/
    void resetGridPathInfo();
    void getExploredGrids(vector<int> &explored_ids);
    bool checkPointRange(const Eigen::Vector3d &pos);
    /******************************************************/
    int getGridStatus(const int &grid_id);

    int getGridNum();

  private:
    void updateGridInfo(const Eigen::Vector3i &id);

    int toAddress(const Eigen::Vector3i &id);
    void adrToIndex(const int &adr, Eigen::Vector3i &idx);
    void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
    void indexToPos(const Eigen::Vector3i &id, const double &inc, Eigen::Vector3d &pos);
    bool insideGrid(const Eigen::Vector3i &id);

    bool isRelevant(const GridInfo &grid);

    shared_ptr<EDTEnvironment> edt_;
    vector<GridInfo> grid_data_;

    vector<int> relevant_id_;
    unordered_map<int, int> relevant_map_;
    bool initialized_;
    vector<int> extra_ids_;

    Eigen::Vector3d resolution_;
    Eigen::Vector3d min_, max_;
    Eigen::Vector3i grid_num_;
    Eigen::Vector3i scale_;   //LZC added

    int level_;

    int min_unknown_, min_frontier_, min_free_;
    double min_unknown_percent_;
    double min_free_percent_;

    double consistent_cost_, inside_ratio_;
    double w_unknown_;

    // Swarm tf
    Eigen::Matrix3d rot_sw_;
    Eigen::Vector3d trans_sw_;

    enum HgridType { ACTIVE = 0, DORMANT =1, DEACTIVE = 2};
    
    int type_;

    friend HGrid;
  };

} // namespace shield
#endif