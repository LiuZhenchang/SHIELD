
#include "pointcloud_filter.h"

void pointcloud_filter::init()
{
    ros::NodeHandle node_("~");

    node_.param("pointcloud_filter/cloud_type", cloud_type_, 1);
    node_.param("pointcloud_filter/min_ray_length", min_ray_length_, 2.0);
    node_.param("pointcloud_filter/max_ray_length", max_ray_length_, 30.0);
    node_.param("pointcloud_filter/leaf_size", leaf_size_, 0.5);
    node_.param("pointcloud_filter/fov_constraint", fov_constraint_, false);
    node_.param("pointcloud_filter/theta_max", theta_max_, 55.0);
    node_.param("pointcloud_filter/theta_min", theta_min_, -7.0);
    node_.param("pointcloud_filter/phi_max", phi_max_, 180.0);
    node_.param("pointcloud_filter/phi_min", phi_min_, -180.0);
    node_.param("pointcloud_filter/frame_id", frame_id_, std::string("map"));

    switch (cloud_type_)
    {
    case 1:
        cloud_sub_1_ = node_.subscribe<sensor_msgs::PointCloud>("/cloud_original", 10, &pointcloud_filter::cloudCallback1, this);
        break;
    case 2:
        cloud_sub_2_ = node_.subscribe<sensor_msgs::PointCloud2>("/cloud_original", 10, &pointcloud_filter::cloudCallback2, this);
        break;
    default:
        ROS_ERROR_STREAM("\033[31m pointcloud_filter: cloud_type_ error \033[0m");
        break;
    }
    cloud_pub_1_ = node_.advertise<sensor_msgs::PointCloud2>("/cloud_filtered_sensor", 10);
    cloud_pub_2_ = node_.advertise<sensor_msgs::PointCloud2>("/cloud_filtered_sensor_inFov", 10);

    theta_max_ = theta_max_ * M_PI / 180.0;
    theta_min_ = theta_min_ * M_PI / 180.0;
    phi_max_ = phi_max_ * M_PI / 180.0;
    phi_min_ = phi_min_ * M_PI / 180.0;
    /***************************** initialize point cloud filtering data ****************************/
    // 1. prepare the AND combined condition
    condition_.reset(new pcl::ConditionAnd<pcl::PointXYZ>());
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Vector3f Zero = Eigen::Vector3f::Zero();

    // 2. distance lower bound: x^2+y^2+z^2 - min^2 >= 0
    comp_min_.reset(new pcl::TfQuadraticXYZComparison<pcl::PointXYZ>());
    comp_min_->setComparisonMatrix(I);                                   // A = I
    comp_min_->setComparisonVector(Zero);                                // v = 0
    comp_min_->setComparisonScalar(-min_ray_length_ * min_ray_length_);  // c = -min^2
    comp_min_->setComparisonOperator(pcl::ComparisonOps::CompareOp::GE); // >= 0
    condition_->addComparison(comp_min_);

    // 3. distance upper bound: x^2+y^2+z^2 - max^2 <= 0
    comp_max_.reset(new pcl::TfQuadraticXYZComparison<pcl::PointXYZ>());
    comp_max_->setComparisonMatrix(I);                                   // A = I
    comp_max_->setComparisonVector(Zero);                                // v = 0
    comp_max_->setComparisonScalar(-max_ray_length_ * max_ray_length_);  // c = -max^2
    comp_max_->setComparisonOperator(pcl::ComparisonOps::CompareOp::LE); // <= 0
    condition_->addComparison(comp_max_);
}

void pointcloud_filter::cloudCallback1(const sensor_msgs::PointCloud::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original_sensor(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered_sensor;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered_sensor_inFov;

    sensor_msgs::PointCloud2 msg2;
    convertPointCloudToPointCloud2(*msg, msg2);
    pcl::fromROSMsg(msg2, *cloud_original);
    cloud_original_sensor = cloud_original;

    cloudFilter(cloud_original_sensor, cloud_filtered_sensor);
    if (cloud_filtered_sensor.empty())
    {
        return;
    }

    // publish filtered sensor cloud
    sensor_msgs::PointCloud2 cloud_msg1;
    pcl::toROSMsg(cloud_filtered_sensor, cloud_msg1);
    cloud_msg1.header = msg->header;
    cloud_msg1.header.frame_id = frame_id_;
    cloud_pub_1_.publish(cloud_msg1);

    if (fov_constraint_)
    {
        insideFov(cloud_filtered_sensor, cloud_filtered_sensor_inFov);
    }
    else
    {
        cloud_filtered_sensor_inFov = cloud_filtered_sensor;
    }

    // publish filtered sensor cloud in Fov
    sensor_msgs::PointCloud2 cloud_msg2;
    pcl::toROSMsg(cloud_filtered_sensor_inFov, cloud_msg2);
    cloud_msg2.header = msg->header;
    cloud_msg2.header.frame_id = frame_id_;
    cloud_pub_2_.publish(cloud_msg2);
}

void pointcloud_filter::cloudCallback2(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original_sensor(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered_sensor;
    pcl::PointCloud<pcl::PointXYZ> cloud_filtered_sensor_inFov;

    pcl::fromROSMsg(*msg, *cloud_original);
    cloud_original_sensor = cloud_original;

    cloudFilter(cloud_original_sensor, cloud_filtered_sensor);
    if (cloud_filtered_sensor.empty())
    {
        return;
    }

    // publish filtered sensor cloud
    sensor_msgs::PointCloud2 cloud_msg1;
    pcl::toROSMsg(cloud_filtered_sensor, cloud_msg1);
    cloud_msg1.header.frame_id = frame_id_;
    cloud_msg1.header.stamp = msg->header.stamp;
    cloud_pub_1_.publish(cloud_msg1);

    if (fov_constraint_)
    {
        insideFov(cloud_filtered_sensor, cloud_filtered_sensor_inFov);
    }
    else
    {
        cloud_filtered_sensor_inFov = cloud_filtered_sensor;
    }

    // publish filtered sensor cloud in Fov
    sensor_msgs::PointCloud2 cloud_msg2;
    pcl::toROSMsg(cloud_filtered_sensor_inFov, cloud_msg2);
    cloud_msg2.header.frame_id = frame_id_;
    cloud_msg2.header.stamp = msg->header.stamp;
    cloud_pub_2_.publish(cloud_msg2);
}

void pointcloud_filter::cloudFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw,
                                    pcl::PointCloud<pcl::PointXYZ> &cloud_filtered)
{
    if (cloud_raw->empty())
    {
        return;
    }
    // initialize point cloud processing objects
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZ>());

    // create a conditional removal filter, filtering by the max/min ray range
    pcl::ConditionalRemoval<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud_raw);
    filter.setCondition(condition_);
    filter.setKeepOrganized(false); // discard indices, return compact output
    filter.filter(*cloud_temp);
    if (cloud_temp->empty())
    {
        return;
    }

    // create a voxel grid (template) filter to downsample the point cloud, point data type is pcl::PointXYZ
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    const float kLeafSize = leaf_size_;
    vg.setLeafSize(kLeafSize, kLeafSize, kLeafSize);
    vg.setInputCloud(cloud_temp); // set the input point cloud, note: a smart pointer to the point cloud object is passed in here
    vg.filter(*cloud_temp1);      // run the filter and output the processed point cloud data, note: a point cloud object is passed in here
    if (cloud_temp1->empty())
    {
        return;
    }

    // create a radius outlier removal filter
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> radius_filter;
    radius_filter.setInputCloud(cloud_temp1);
    radius_filter.setRadiusSearch(1.0);
    radius_filter.setMinNeighborsInRadius(5);
    radius_filter.filter(cloud_filtered);
}

void pointcloud_filter::insideFov(const pcl::PointCloud<pcl::PointXYZ> &cloud_original,
                                  pcl::PointCloud<pcl::PointXYZ> &cloud_filtered)
{
    if (theta_max_ < theta_min_)
    {
        ROS_ERROR_STREAM("pointcloud_filter::insideFov: theta_max_ < theta_min_");
        return;
    }
    if (phi_max_ < phi_min_)
    {
        ROS_ERROR_STREAM("pointcloud_filter::insideFov: phi_max_ < phi_min_");
        return;
    }
    for (const auto &point : cloud_original)
    {
        double distance = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        if (distance > max_ray_length_ || distance < min_ray_length_)
        {
            continue;
        }
        double phi = atan2(point.y, point.x);    // [-π, π]
        double theta = asin(point.z / distance); // [-π/2, π/2]
        if (theta > theta_max_ || theta < theta_min_)
        {
            continue;
        }
        if (phi < phi_min_ || phi > phi_max_)
        {
            continue;
        }
        cloud_filtered.push_back(point);
    }
}
