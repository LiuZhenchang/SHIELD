#include <plan_env/sdf_map.h>
#include <plan_env/map_ros.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>

namespace shield
{
    MapROS::MapROS()
    {
    }

    MapROS::~MapROS()
    {
        thread_running_ = false;
        if (callback_thread_.joinable())
        {
            callback_thread_.join();
        }

        thread_map_running_ = false;
        if (callback_map_thread_.joinable()) 
        {
            callback_map_thread_.join();
        }
    }

    void MapROS::setMap(SDFMap *map)
    {
        this->map_ = map;
    }

    void MapROS::threadFunc()
    {
        // printThreadId("threadFunc");
        ros::Rate rate(500); // High-frequency processing
        while (thread_running_ && ros::ok())
        {
            custom_queue_.callAvailable(ros::WallDuration(0.01));
            rate.sleep();
        }
    }

    void MapROS::threadMapFunc()
    {
        // printThreadId("threadOdomFunc");
        ros::Rate rate(500); // High-frequency processing
        while (thread_map_running_ && ros::ok())
        {
            custom_map_queue_.callAvailable(ros::WallDuration(0.01));
            rate.sleep();
        }
    }

    void MapROS::printThreadId(const std::string &context)
    {
        std::thread::id this_id = std::this_thread::get_id();
        std::string showText = "\033[45;37m" + context + " running in thread: " + "\033[0m";
    }

    void MapROS::init()
    {
        node_.param("map_ros/visualization_truncate_height", visualization_truncate_height_, -0.1);
        node_.param("map_ros/visualization_truncate_low", visualization_truncate_low_, -0.1);
        node_.param("map_ros/show_all_map", show_all_map_, false);
        node_.param("map_ros/frame_id", frame_id_, string("map"));
        node_.param("map_ros/cloud_frame_type", cloud_frame_type_, 0);
        node_.param("sdf_map/max_ray_length", max_ray_length_, 30.0);
        node_.param("sdf_map/min_ray_length", min_ray_length_, 0.5);
        node_.param("sdf_map/leaf_size", leaf_size_, 1.0);

        local_updated_ = false;
        esdf_need_update_ = false;
        esdf_num_ = 0;
        esdf_time_ = 0.0;
        max_esdf_time_ = 0.0;

        esdf_timer_ = node_.createTimer(ros::Duration(0.5), &MapROS::updateESDFCallback, this);
        // vis_timer_ = node_.createTimer(ros::Duration(0.5), &MapROS::visCallback, this);

        ros::NodeHandle nh_map_custom;
        nh_map_custom.setCallbackQueue(&custom_map_queue_);
        vis_timer_ = nh_map_custom.createTimer(ros::Duration(0.5), &MapROS::visCallback, this);
        thread_map_running_ = true;
        callback_map_thread_ = boost::thread(&MapROS::threadMapFunc, this);

        map_inflate_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/inflate_occupancy_all", 10);
        map_quality_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/quality_all", 10);
        map_free_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/free_all", 10);
        map_unknown_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/unknown_all", 10);
        map_hybrid_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/sdf_map/hybrid_all", 10);
        cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/lidar_filtered_global", 10);




        /***************************** Initialize point cloud filtering data ****************************/
        // 1. Prepare the AND combined condition
        condition_.reset(new pcl::ConditionAnd<pcl::PointXYZ>());
        Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
        Eigen::Vector3f Zero = Eigen::Vector3f::Zero();

        // 2. Distance lower bound: x^2+y^2+z^2 - min^2 >= 0
        comp_min_.reset(new pcl::TfQuadraticXYZComparison<pcl::PointXYZ>());
        comp_min_->setComparisonMatrix(I);                                   // A = I
        comp_min_->setComparisonVector(Zero);                                // v = 0
        comp_min_->setComparisonScalar(-min_ray_length_ * min_ray_length_);  // c = -min^2
        comp_min_->setComparisonOperator(pcl::ComparisonOps::CompareOp::GE); // >= 0
        condition_->addComparison(comp_min_);

        // 3. Distance upper bound: x^2+y^2+z^2 - max^2 <= 0
        comp_max_.reset(new pcl::TfQuadraticXYZComparison<pcl::PointXYZ>());
        comp_max_->setComparisonMatrix(I);                                   // A = I
        comp_max_->setComparisonVector(Zero);                                // v = 0
        comp_max_->setComparisonScalar(-max_ray_length_ * max_ray_length_);  // c = -max^2
        comp_max_->setComparisonOperator(pcl::ComparisonOps::CompareOp::LE); // <= 0
        condition_->addComparison(comp_max_);


        ros::NodeHandle nh_custom;
        nh_custom.setCallbackQueue(&custom_queue_);

        cloud_sub_sync_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_custom, "/map_ros/cloud", 1));
        odom_sub_sync_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(nh_custom, "/map_ros/odom", 20));
        odom_cloud_sync_.reset(new message_filters::Synchronizer<SyncPolicyOdomCloud>(SyncPolicyOdomCloud(20), *odom_sub_sync_, *cloud_sub_sync_));
        odom_cloud_sync_->registerCallback(boost::bind(&MapROS::odomCloudCallback, this, _1, _2));

        // Start the thread that processes the custom queue
        thread_running_ = true;
        callback_thread_ = boost::thread(&MapROS::threadFunc, this);


        cloud_lidar_sensor_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_lidar_local_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_lidar_global_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        map_initialized_ = false;

        SE3_Matrix_L2B_ = map_->percep_utils_lidar_->getSE3MatrixL2B();

    }



    void MapROS::visCallback(const ros::TimerEvent &e)
    {
        if (show_all_map_)
        {
            // Limit the frequency of all map
            static double tpass = 0.0;
            tpass += (e.current_real - e.last_real).toSec();
            if (tpass > 0.1)
            {
                publishMapAll();
                publishQuality();
                tpass = 0.0;
            }
        }
    }

    void MapROS::updateESDFCallback(const ros::TimerEvent & /*event*/)
    {
        if (!esdf_need_update_)
        {
            return;
        }

        auto t1 = ros::Time::now();
        map_->updateESDF3d();

        esdf_need_update_ = false;
        auto t2 = ros::Time::now();

        esdf_time_ += (t2 - t1).toSec();
        max_esdf_time_ = max(max_esdf_time_, (t2 - t1).toSec());
        esdf_num_++;
    }

    void MapROS::odomCallback(const nav_msgs::OdometryConstPtr &odom)
    {

        Eigen::Quaterniond odom_q_;
        odom_q_.w() = odom->pose.pose.orientation.w;
        odom_q_.x() = odom->pose.pose.orientation.x;
        odom_q_.y() = odom->pose.pose.orientation.y;
        odom_q_.z() = odom->pose.pose.orientation.z;

        odom_pos_.x() = odom->pose.pose.position.x;
        odom_pos_.y() = odom->pose.pose.position.y;
        odom_pos_.z() = odom->pose.pose.position.z;

        SE3_Matrix_B2G_ = Eigen::Isometry3d::Identity();
        SE3_Matrix_B2G_.translation() = odom_pos_;
        SE3_Matrix_B2G_.linear() = odom_q_.toRotationMatrix();
        SE3_Matrix_L2G_ = SE3_Matrix_B2G_ * SE3_Matrix_L2B_;
        sensor_pos_ = SE3_Matrix_L2G_.translation();
        map_->percep_utils_lidar_->setPose(odom_pos_, odom_q_);
    }

    void MapROS::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        if (!map_initialized_)
        {
            ROS_ERROR_STREAM("MapROS: SDFMap is not initialized");
            return;
        }
        cloud_lidar_sensor_->clear();
        cloud_lidar_local_->clear();
        cloud_lidar_global_->clear();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::fromROSMsg(*cloud, *cloud_temp);
        if (cloud_temp->points.empty())
        {
            return;
        }

        switch(cloud_frame_type_)
        {
            case 0: // lidar frame
                *cloud_lidar_sensor_ = *cloud_temp;
                break;
            case 1: // body frame
                pcl::transformPointCloud(*cloud_temp, *cloud_lidar_sensor_, SE3_Matrix_L2B_.inverse().matrix());
                break;
            case 2: // global frame
                pcl::transformPointCloud(*cloud_temp, *cloud_lidar_sensor_, SE3_Matrix_L2G_.inverse().matrix());
                break;
            default:
                ROS_ERROR_STREAM("MapROS: cloudCallback frame_type error");
                return;
        }

        pcl::PointCloud<pcl::PointXYZ> filtered_points;
        cloudFilter(cloud_lidar_sensor_, filtered_points);

        pcl::transformPointCloud(filtered_points, *cloud_lidar_local_, SE3_Matrix_L2B_.matrix());
        pcl::transformPointCloud(filtered_points, *cloud_lidar_global_, SE3_Matrix_L2G_.matrix());
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud_lidar_global_, cloud_msg);
        cloud_msg.header.frame_id = frame_id_;
        cloud_msg.header.stamp = cloud->header.stamp;
        cloud_pub_.publish(cloud_msg);
        
        // Fusion of multiple frames of continuous point clouds in the sensor frame 
        pointcloud_buffer_.push_back(*cloud_lidar_sensor_);
        if (pointcloud_buffer_.size() > map_->pointcloud_buffer_length_)
        {
            pointcloud_buffer_.erase(pointcloud_buffer_.begin());
        }
        cloud_merge_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        for (int i = 0; i < pointcloud_buffer_.size(); i++)
        {
            *cloud_merge_ += pointcloud_buffer_[i];
        }
        // Project the merged pointcloud onto a sphere
        map_->percep_utils_lidar_->updateProjection(cloud_merge_, map_->mp_->resolution_);
        map_->inputPointCloud(*cloud_lidar_global_, sensor_pos_);
        
        if (local_updated_)
        {
            map_->clearAndInflateLocalMap();
            esdf_need_update_ = true;
            local_updated_ = false;
        }
    }

    void MapROS::odomCloudCallback(const nav_msgs::Odometry::ConstPtr &odom,
                                   const sensor_msgs::PointCloud2::ConstPtr &cloud)
    {
        printThreadId("odomCloudCallback");
        odomCallback(odom);
        cloudCallback(cloud);
    }


    void MapROS::publishMapAll()
    {
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud1; // inflate occupancy voxels
        pcl::PointCloud<pcl::PointXYZ> cloud2; // unknow voxels
        pcl::PointCloud<pcl::PointXYZ> cloud3; // free voxels
        pcl::PointCloud<pcl::PointXYZ> cloud4; // hybrid voxels

        Eigen::Vector3i min_idx, max_idx;
        
        //Draw all Voxels in the map, not only those in the update box
        min_idx = Eigen::Vector3i(0,0,0);
        max_idx = Eigen::Vector3i(map_->mp_->map_voxel_num_(0), map_->mp_->map_voxel_num_(1), map_->mp_->map_voxel_num_(2));

        for (int x = min_idx[0]; x < max_idx[0]; ++x)
            for (int y = min_idx[1]; y < max_idx[1]; ++y)
                for (int z = min_idx[2]; z < max_idx[2]; ++z)
                {
                    Eigen::Vector3i id = {x, y, z};
                    Eigen::Vector3d pos;
                    int state = map_->getOccupancy(id);
                    if (map_->getInflateOccupancy(id) == 1)
                    {
                        map_->indexToPos(id, pos);
                        if (pos(2) > visualization_truncate_height_)
                            continue;
                        if (pos(2) < visualization_truncate_low_)
                            continue;
                        pt.x = pos(0);
                        pt.y = pos(1);
                        pt.z = pos(2);
                        cloud1.push_back(pt);
                    }

                    if (state == map_->UNKNOWN)
                    {
                        Eigen::Vector3d pos;
                        map_->indexToPos(id, pos);
                        if (pos(2) > visualization_truncate_height_)
                            continue;
                        if (pos(2) < visualization_truncate_low_)
                            continue;
                        pt.x = pos(0);
                        pt.y = pos(1);
                        pt.z = pos(2);
                        cloud2.push_back(pt);
                    }

                    if (state == map_->FREE)
                    {
                        Eigen::Vector3d pos;
                        map_->indexToPos(id, pos);
                        if (pos(2) > visualization_truncate_height_)
                            continue;
                        if (pos(2) < visualization_truncate_low_)
                            continue;
                        pt.x = pos(0);
                        pt.y = pos(1);
                        pt.z = pos(2);
                        cloud3.push_back(pt);

                        bool hybrid = (isNeighborUnknown(id) && isNeighborWellObserved(id));

                        if (hybrid)
                        {
                            Eigen::Vector3d pos;
                            map_->indexToPos(id, pos);
                            if (pos(2) > visualization_truncate_height_)
                                continue;
                            if (pos(2) < visualization_truncate_low_)
                                continue;
                            pt.x = pos(0);
                            pt.y = pos(1);
                            pt.z = pos(2);
                            cloud4.push_back(pt);
                        }
                    }
                }

        cloud1.width = cloud1.points.size();
        cloud1.height = 1;
        cloud1.is_dense = true;
        cloud1.header.frame_id = frame_id_;
        sensor_msgs::PointCloud2 cloud_msg1;
        pcl::toROSMsg(cloud1, cloud_msg1);
        map_inflate_pub_.publish(cloud_msg1);

        cloud2.width = cloud2.points.size();
        cloud2.height = 1;
        cloud2.is_dense = true;
        cloud2.header.frame_id = frame_id_;
        sensor_msgs::PointCloud2 cloud_msg2;
        pcl::toROSMsg(cloud2, cloud_msg2);
        map_unknown_pub_.publish(cloud_msg2);

        cloud3.width = cloud3.points.size();
        cloud3.height = 1;
        cloud3.is_dense = true;
        cloud3.header.frame_id = frame_id_;
        sensor_msgs::PointCloud2 cloud_msg3;
        pcl::toROSMsg(cloud3, cloud_msg3);
        map_free_pub_.publish(cloud_msg3);

        cloud4.width = cloud4.points.size();
        cloud4.height = 1;
        cloud4.is_dense = true;
        cloud4.header.frame_id = frame_id_;
        sensor_msgs::PointCloud2 cloud_msg4;
        pcl::toROSMsg(cloud4, cloud_msg4);
        map_hybrid_pub_.publish(cloud_msg4);
    }

    void MapROS::publishQuality()
    {
        // printThreadId("publishQuality");
        pcl::PointXYZRGB pt;
        pcl::PointCloud<pcl::PointXYZRGB> cloud1;
        Eigen::Vector3i min_idx, max_idx;
        map_->posToIndex(map_->md_->all_min_, min_idx);
        map_->posToIndex(map_->md_->all_max_, max_idx);

        map_->boundIndex(min_idx);
        map_->boundIndex(max_idx);

        for (int x = min_idx[0]; x <= max_idx[0]; ++x)
            for (int y = min_idx[1]; y <= max_idx[1]; ++y)
                for (int z = min_idx[2]; z <= max_idx[2]; ++z)
                {
                    if (map_->md_->occupancy_buffer_[map_->toAddress(x, y, z)] >
                        map_->mp_->min_occupancy_log_)
                    {
                        Eigen::Vector3d pos;
                        int color =
                            // floor((map_->md_->min_observed_dist_[map_->toAddress(x, y, z)] /
                            //         observe_update_dist) *
                            //         510);
                            floor(((1.0 - map_->md_->max_observed_angle_[map_->toAddress(x, y, z)]) /
                                   1.0) *
                                  510);

                        map_->indexToPos(Eigen::Vector3i(x, y, z), pos);
                        if (pos(2) > visualization_truncate_height_)
                            continue;
                        if (pos(2) < visualization_truncate_low_)
                            continue;
                        pt.x = pos(0);
                        pt.y = pos(1);
                        pt.z = pos(2);
                        pt.r = color < 255 ? color : 255;
                        pt.g = color > 255 ? 510 - color : 255;
                        pt.b = 0;
                        // pt.r = 255 - color;
                        // pt.g = color;
                        // pt.b = 0;
                        cloud1.push_back(pt);
                    }
                }
        cloud1.width = cloud1.points.size();
        cloud1.height = 1;
        cloud1.is_dense = true;
        cloud1.header.frame_id = frame_id_;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud1, cloud_msg);
        map_quality_pub_.publish(cloud_msg);
    };

    bool MapROS::isNeighborUnknown(const Eigen::Vector3i &voxel)
    {
        // At least one neighbor is unknown
        auto nbrs = allNeighbors(voxel, 1);
        for (auto nbr : nbrs)
        {
            if (map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
                return true;
        }
        return false;
    }

    vector<Eigen::Vector3i> MapROS::allNeighbors(const Eigen::Vector3i &voxel, const int step)
    {
        vector<Eigen::Vector3i> neighbors;
        Eigen::Vector3i tmp;
        for (int x = -step; x <= step; ++x)
            for (int y = -step; y <= step; ++y)
                for (int z = -step; z <= step; ++z)
                {
                    if (x == 0 && y == 0 && z == 0)
                        continue;
                    tmp = voxel + Eigen::Vector3i(x, y, z);
                    if (map_->isInMap(tmp))
                    {
                        neighbors.push_back(tmp);
                    }
                }
        return neighbors;
    }

    bool MapROS::isNeighborWellObserved(const Eigen::Vector3i &voxel)
    {
        // At least one neighbor is Well Observed
        auto nbrs = allNeighbors(voxel, 1);
        for (auto nbr : nbrs)
        {
            if (map_->getOccupancy(nbr) == SDFMap::OCCUPIED && map_->getObservedAngle(map_->toAddress(nbr)) >= 0.5)
                return true;
        }
        return false;
    }

    void MapROS::cloudFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw,
                                        pcl::PointCloud<pcl::PointXYZ> &cloud_filtered)
    {
        if (cloud_raw->empty())
        {
            return;
        }
        // Point cloud processing object initialization
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZ>());

        // Create a conditional removal filter object, filtering by the min/max ray range
        pcl::ConditionalRemoval<pcl::PointXYZ> filter;
        filter.setInputCloud(cloud_raw);
        filter.setCondition(condition_);
        filter.setKeepOrganized(false); // Discard indices and return a compact output
        filter.filter(*cloud_temp);
        if (cloud_temp->empty())
        {
            return;
        }

        // Create a voxel grid (templated) filter object to downsample the point cloud; the point data type is pcl::PointXYZ
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        const float kLeafSize = leaf_size_;
        vg.setLeafSize(kLeafSize, kLeafSize, kLeafSize);
        vg.setInputCloud(cloud_temp); // Set the input point cloud; note: a smart pointer to the point cloud object is passed here
        vg.filter(*cloud_temp1);      // Perform filtering and output the processed point cloud data; note: a point cloud object is passed here
        if (cloud_temp1->empty())
        {
            return;
        }

        // Create a radius outlier removal filter object
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
        radius_filter.setInputCloud(cloud_temp1);
        radius_filter.setRadiusSearch(1.0);
        radius_filter.setMinNeighborsInRadius(5);
        radius_filter.filter(cloud_filtered);
    }
}