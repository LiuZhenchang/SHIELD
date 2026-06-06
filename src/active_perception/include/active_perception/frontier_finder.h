#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <geometry_msgs/PoseStamped.h>
#include <list>
#include <memory>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <queue>
#include <ros/ros.h>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <path_searching/astar2.h>


using Eigen::Vector3d;
using std::list;
using std::pair;
using std::priority_queue;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

namespace shield {

class EDTEnvironment;
class PerceptionUtils_Lidar;


// Viewpoint to cover a frontier cluster
struct Viewpoint {
    // Position and heading
    Vector3d pos_;
    double yaw_;
    // Fraction of the cluster that can be covered
    // double fraction_;
    int visib_num_;
    double quality_;
  };

// A frontier cluster, the viewpoints to cover it
struct Frontier {
    // Complete voxels belonging to the cluster
    vector<Vector3d> cells_;
    // down-sampled voxels filtered by voxel grid filter
    vector<Vector3d> filtered_cells_;
    // Average position of all voxels
    Vector3d average_;
    // Average normal of all voxels
    Vector3d normal_;
    // Idx of frontier
    int id_;
    // Idx of the cluster it belongs to
    int clu_id_;
    int type_;
    bool divided;
    bool is_target_ = false;
    // Viewpoints that can cover the cluster
    vector<Viewpoint> viewpoints_;
    // Bounding box of cluster, center & 1/2 side length
    Vector3d box_min_, box_max_;
    // Path and cost from this cluster to other clusters
    list<vector<Vector3d>> paths_;
    list<double> costs_;
  };

  class FrontierFinder {
    public:
    enum FrontierTYPE { FREEUNKNOWNFTR = 0, QUALITYFTR =1};
    FrontierFinder(const shared_ptr<EDTEnvironment> &edt, ros::NodeHandle &nh);
    ~FrontierFinder();
    void drawFrontier(Frontier &frontier);

    void searchFrontiers(Eigen::Vector3d cur_pos, double cur_yaw);
    void computeFrontiersToVisit(Eigen::Vector3d cur_pos);
    void getFrontiers(vector<vector<Vector3d>> &clusters);
    void mygetFrontiers(vector<Frontier> &clusters);


    void getViewpoints(vector<vector<Viewpoint>> &clusters);
    void getTopViewpointsInfo(const Vector3d& cur_pos, vector<Vector3d>& points, vector<double>& yaws,
      vector<Vector3d>& averages);
    void getboundary(vector<Vector3d> &clusters);
    void setTargetFrontier(int id);
    void resetTargetFrontier();
    void deactiveTargetFrontier(Eigen::Vector3d pos, double yaw);
    bool haveTargetFrontier();

    void testmap();//测试拿到的地图数据是否正确

    ros::Publisher frontier_pub_;

    void updateFrontierCostMatrix();
    void getFullCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw, Eigen::MatrixXd& mat);
    void getPathForTour(const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path);
    void getSwarmCostMatrix(const vector<Vector3d> &positions, const vector<Vector3d> &velocities, const vector<double> yaws, Eigen::MatrixXd &mat);
    void getSwarmCostMatrix(const vector<Vector3d> &positions, const vector<Vector3d> &velocities, const vector<double> &yaws, const vector<int> &ftr_ids, const vector<Eigen::Vector3d> &grid_pos, Eigen::MatrixXd &mat);

    private:
    void splitLargeFrontiers(list<Frontier>& frontiers);
    bool splitIn3D(const Frontier &frontier, list<Frontier> &splits);
    void sampleViewpoints(Frontier &frontier);
    int countVisibleCells(const Vector3d &pos, const double &yaw,
                          const vector<Vector3d> &cluster, const int ftr_type,
                          double &quality_av, bool is_target);
    bool isNearObstacle(const Vector3d &pos);    
    vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i &voxel);
    vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i &voxel);
    vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i &voxel);
    vector<Eigen::Vector3i> allNeighbors1(const Eigen::Vector3i &voxel);
  
    bool isNeighborUnknown(const Eigen::Vector3i &voxel);
    bool isNeighborUnknown1(const Eigen::Vector3i &voxel);
    bool isNeighborOccupy(const Eigen::Vector3i &voxel);
    bool isNeighborFree(const Eigen::Vector3i &voxel);
    bool isNeighborWellObserved(const Eigen::Vector3i &voxel);


    // Wrapper of sdf map
    int toadr(const Eigen::Vector3i &idx);
    bool knownfree(const Eigen::Vector3i &idx);
    bool knownoccupy(const Eigen::Vector3i &idx);
    bool unknown(const Eigen::Vector3i &idx);


    void wrapYaw(double &yaw);

    bool haveOverlap(const Vector3d &min1, const Vector3d &max1, const Vector3d &min2, const Vector3d &max2);
    bool isFrontierChanged(const Frontier &ft, Eigen::Vector3d &cur_pos, double &cur_yaw);

    void expandFrontier(const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */);
    void computeFrontierInfo(Frontier& frontier);
    void drawDebugInfo(const vector<Eigen::Vector3d>& pts, int pub_id);


    bool insert_frontier = false; // 标记是否有frontier被插入
    bool deactive_target_frontier_ = false; //强制删除目标frontier


    // Data
    vector<char> frontier_flag_;
    list<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_;
    vector<int> removed_ids_;
    int first_new_frt_;
    int reach_count_;
    list<Frontier>::iterator first_new_ftr_;
    
    int frts_num_after_remove_;
    Frontier next_frontier_;

    double searchPath(const Vector3d& p1, const Vector3d& p2, vector<Vector3d>& path);
    double computeCost(const Vector3d& p1, const Vector3d& p2, const double& y1, const double& y2, const Vector3d& v1, const double& yd1, vector<Vector3d>& path);


    // Params
    int cluster_min_;
    int down_sample_;
    int search_step_;
    int min_visib_num_;
    int target_frt_id_;
    int candidate_rnum_, candidate_hnum_;

    double cluster_size_xy_, cluster_size_z_;
    double candidate_rmax_, candidate_rmin_, candidate_dphi_;
    double min_candidate_dist_, min_candidate_clearance_;
    double candidate_hmax_, candidate_hmin_; //柱坐标高方向撒点
    double min_view_finish_fraction_;
    double resolution_;
    
    double min_visib_percent_; // 最小可见性百分比
    double division_ratio_;
    double frt_cluster_radius_;


    string tsp_dir_;

    // Utils
    ros::Publisher debug_pts_1_, debug_pts_2_;
    shared_ptr<EDTEnvironment> edt_env_;
    shared_ptr<PerceptionUtils_Lidar> percep_utils_lidar_;
    unique_ptr<RayCaster> raycaster_;
    unique_ptr<Astar> astar_;
    double vm_;
    double am_;
    double yd_;
    double ydd_;
    double w_dir_;


    double threshold_max_ = 0.9;
    double threshold_min_ = 0.5;
    double dist_threshold_ = 5.0;
    double min_quality_av_ = 0.5;
    double well_observed_ = 0.9;
    double ignore_height_;
    double ignore_height_up_;

    vector<Vector3d> boundary_cells_;

  };

}





#endif
