#ifndef _MAP_ROS_H
#define _MAP_ROS_H

#include <ros/ros.h>
#include <memory>
#include <thread>
#include <random>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

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
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

using std::default_random_engine;
using std::normal_distribution;
using std::shared_ptr;

namespace shield
{
        class SDFMap;

        class MapROS
        {
        public:
                MapROS();
                ~MapROS();
                void setMap(SDFMap *map);
                void init();

        private:
                void odomCloudCallback(const nav_msgs::Odometry::ConstPtr &odom,
                                       const sensor_msgs::PointCloud2::ConstPtr &cloud);
                void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud);
                void cloudCallback_flight(const sensor_msgs::PointCloud2ConstPtr &cloud);

                void odomCallback(const nav_msgs::OdometryConstPtr &odom);
                void updateESDFCallback(const ros::TimerEvent &e);
                void visCallback(const ros::TimerEvent &e);

                void publishMapAll();
                void publishQuality();

                bool isNeighborUnknown(const Eigen::Vector3i &voxel);
                bool isNeighborWellObserved(const Eigen::Vector3i &voxel);
                vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i &voxel, const int step);

                SDFMap *map_;

                ros::NodeHandle node_;
                ros::Subscriber cloud_sub_;
                ros::Subscriber odom_sub_;
                ros::Publisher cloud_pub_;

                // 同步回调随体系雷达点云和里程计
                shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_sync_;
                shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_sync_;

                typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2>
                    SyncPolicyOdomCloud;
                // typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2>
                //     SyncPolicyOdomCloud;
                typedef shared_ptr<message_filters::Synchronizer<SyncPolicyOdomCloud>>
                    SynchronizerOdomCloud;

                SynchronizerOdomCloud odom_cloud_sync_;

                ros::CallbackQueue custom_queue_; // 自定义回调队列
                boost::thread callback_thread_;   // 处理队列的线程
                bool thread_running_;             // 线程运行标志
                bool map_initialized_;

                // 线程执行函数
                void threadFunc();
                // 打印线程id
                void printThreadId(const std::string &context);
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar_sensor_;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar_local_;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lidar_global_;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merge_;
                vector<pcl::PointCloud<pcl::PointXYZ>> pointcloud_buffer_; 

                ros::Publisher map_inflate_pub_;
                ros::Publisher map_quality_pub_;
                ros::Publisher map_free_pub_;
                ros::Publisher map_unknown_pub_;
                ros::Publisher map_hybrid_pub_;
                ros::Timer vis_timer_;
                ros::Timer esdf_timer_;

                double visualization_truncate_height_, visualization_truncate_low_;
                std::string frame_id_;
                bool show_all_map_;
                bool local_updated_;
                bool esdf_need_update_;

                double esdf_time_;
                double max_esdf_time_;
                int esdf_num_;
                int cloud_frame_type_; // 0: lidar frame, 1: body frame, 2: global frame

                Eigen::Isometry3d SE3_Matrix_B2G_;
                Eigen::Isometry3d SE3_Matrix_L2B_;
                Eigen::Isometry3d SE3_Matrix_L2G_;

                Eigen::Quaterniond odom_q_;
                Eigen::Vector3d odom_pos_;
                Eigen::Vector3d odom_euler_;
                Eigen::Vector3d sensor_pos_;

                friend SDFMap;

                double max_ray_length_, min_ray_length_, leaf_size_;
                
                void cloudFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw,
                                pcl::PointCloud<pcl::PointXYZ> &cloud_filtered);

                // 点云滤波条件
                pcl::ConditionAnd<pcl::PointXYZ>::Ptr condition_;
                pcl::TfQuadraticXYZComparison<pcl::PointXYZ>::Ptr comp_min_;
                pcl::TfQuadraticXYZComparison<pcl::PointXYZ>::Ptr comp_max_;

                ros::CallbackQueue custom_map_queue_;  // 自定义回调队列
                boost::thread callback_map_thread_;    // 处理队列的线程
                bool thread_map_running_;

                void threadMapFunc();


        };

}

#endif