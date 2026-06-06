#ifndef _PERCEPTION_UTILS_H_
#define _PERCEPTION_UTILS_H_

#include <ros/ros.h>

#include <Eigen/Eigen>

#include <iostream>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <unordered_set>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace shield
{
  class PerceptionUtils_Lidar
  {
  public:
    PerceptionUtils_Lidar(ros::NodeHandle &nh);
    ~PerceptionUtils_Lidar() {}

    void drawSphere();
    void drawProjection();
    void setPose(const Eigen::Vector3d &odom_pos, const Eigen::Quaterniond &odom_orient);
    bool insideFOV(const Eigen::Vector3d &point_world);
    void testinsideFOV();
    void posToSphere(const Eigen::Vector3d &point, double &r, double &phi, double &theta);
    int sphereToIndex(const double &r, const double &phi, const double &theta);
    void indexToSphere(const int &id, double &r, double &phi, double &theta);
    Eigen::Vector3d indexToGlobal(const int &id);

    void updateProjection(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double size);
    void project_lidar_to_global(const double &x, const double &y, const double &z);

    Eigen::Vector3d getGlobalPos(const int &id);
    double getDistance(const int &id);
    int getNumber(const int &id);
    int getMaxIndex();
    bool neighborNumber(const int &id);
    void resetProjection();

    // 返回激光雷达安装的SE3矩阵
    Eigen::Isometry3d getSE3MatrixL2B();

    string calibration_dir_;
    void count_visible_id();
    void count_visible_id_with_intensity(const pcl::PointCloud<pcl::PointXYZI> &cloud);

    void readDetectiveData();

    bool iscalibrate();
    bool IsIdDeactivated(int id);

    std::vector<int> deactive_id_;
    std::unordered_set<int> deactive_set_; // 用于存储deactive_id_中的id，用于判断是否在deactive_id_中

  private:
    /* Params */
    // Sensing range of camera
    double max_radius_, min_radius_;
    // Mounting angles of camera or lidar
    double mount_yaw_, mount_pitch_, mount_roll_;

    double theta_min_; // 纬度最小值（弧度）
    double theta_max_; // 纬度最大值（默认上半球）
    double phi_min_;   // 经度最小值（弧度）
    double phi_max_;   // 经度最大值（默认完整圆周）

    // 球面步长
    double d_theta_;
    double d_phi_;

    int search_radius_; // 球面格子搜索半径

    ros::Publisher drawer_;
    ros::Publisher drawer1_;
    ros::Publisher drawer2_;

    visualization_msgs::Marker marker_;
    visualization_msgs::Marker text_marker;

    // Current camera pos and attitude
    Vector3d odom_pos_;
    Eigen::Quaterniond odom_orient_;

    // Horizontal field-of-view in degrees
    int Hsize_;
    // Vertical field-of-view in degrees
    int Vsize_;

    std::vector<double> distance_;
    std::vector<int> num_;
    std::vector<Eigen::Vector3d> pos_lidar_; // lidar坐标系下每个格子的位置

    const double epsilon = 1e-10; // 浮点精度容差

    Eigen::Vector3d lidar_trans_;

    // 雷达安装的SE3矩阵
    Eigen::Isometry3d SE3_matrix_lidar_;
    // 飞机里程计的SE3矩阵
    Eigen::Isometry3d SE3_Matrix_odom_;

    std::string frame_id_;

    bool read_calibration_; // 是否校准lidar
    int calibration_num_threshold_;
    double pos_threshold_;
  };

} // namespace shield
#endif
