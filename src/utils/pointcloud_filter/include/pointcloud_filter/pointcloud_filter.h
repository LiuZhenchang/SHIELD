#include <iostream>
#include <ros/ros.h>
#include <Eigen/Eigen>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

class pointcloud_filter
{
public:
    pointcloud_filter(){};
    void init();

private:
    void cloudCallback1(const sensor_msgs::PointCloud::ConstPtr &msg);    
    void cloudCallback2(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void cloudFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original,
                        pcl::PointCloud<pcl::PointXYZ> &cloud_filtered);
    void insideFov(const pcl::PointCloud<pcl::PointXYZ> &cloud_original,
                        pcl::PointCloud<pcl::PointXYZ> &cloud_filtered);
    
    // parameters
    double min_ray_length_;
    double max_ray_length_;
    double leaf_size_;
    double theta_max_;
    double theta_min_;
    double phi_max_;
    double phi_min_;
    int cloud_type_;
    bool fov_constraint_;
    std::string frame_id_;

    ros::NodeHandle node_;
    ros::Publisher cloud_pub_1_;
    ros::Publisher cloud_pub_2_;
    ros::Subscriber cloud_sub_1_;
    ros::Subscriber cloud_sub_2_;

    pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition_;
    pcl::TfQuadraticXYZComparison<pcl::PointXYZ>::Ptr comp_min_;
    pcl::TfQuadraticXYZComparison<pcl::PointXYZ>::Ptr comp_max_;
};