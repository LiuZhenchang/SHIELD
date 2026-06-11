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

    // Returns the SE3 matrix of the lidar mounting
    Eigen::Isometry3d getSE3MatrixL2B();

    string calibration_dir_;
    void count_visible_id();
    void count_visible_id_with_intensity(const pcl::PointCloud<pcl::PointXYZI> &cloud);

    void readDetectiveData();

    bool iscalibrate();
    bool IsIdDeactivated(int id);

    std::vector<int> deactive_id_;
    std::unordered_set<int> deactive_set_; // Stores the ids from deactive_id_, used to check whether an id is in deactive_id_

  private:
    /* Params */
    // Sensing range of camera
    double max_radius_, min_radius_;
    // Mounting angles of camera or lidar
    double mount_yaw_, mount_pitch_, mount_roll_;

    double theta_min_; // Minimum latitude (radians)
    double theta_max_; // Maximum latitude (defaults to upper hemisphere)
    double phi_min_;   // Minimum longitude (radians)
    double phi_max_;   // Maximum longitude (defaults to a full circle)

    // Spherical step size
    double d_theta_;
    double d_phi_;

    int search_radius_; // Search radius in spherical grid cells

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
    std::vector<Eigen::Vector3d> pos_lidar_; // Position of each grid cell in the lidar coordinate frame

    const double epsilon = 1e-10; // Floating-point precision tolerance

    Eigen::Vector3d lidar_trans_;

    // SE3 matrix of the lidar mounting
    Eigen::Isometry3d SE3_matrix_lidar_;
    // SE3 matrix of the aircraft odometry
    Eigen::Isometry3d SE3_Matrix_odom_;

    std::string frame_id_;

    bool read_calibration_; // Whether to calibrate the lidar
    int calibration_num_threshold_;
    double pos_threshold_;
  };

} // namespace shield
#endif
