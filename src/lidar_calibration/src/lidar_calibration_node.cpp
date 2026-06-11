#include <ros/ros.h>
#include <lidar_calibration/lidar_calibration_node.h>

using namespace shield;

int count = 0;
int calibration_num; //number of calibration frames
shared_ptr<PerceptionUtils_Lidar> percep_utils_lidar_;

// pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_norm_(new pcl::PointCloud<pcl::PointXYZI>());

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_norm_;

sensor_msgs::PointCloud2 output_msg;
ros::Publisher pointcloud_norm_pub_;

void scanCallback(const sensor_msgs::PointCloud2::ConstPtr &scan);

int main(int argc, char** argv) {
    ros::init(argc, argv, "lidar_calibration_node");
    ros::NodeHandle nh("~");

    nh.param("lidar_calibration/calibration_num", calibration_num, 600);//number of calibration frames

    // percep_utils_lidar_.reset(new PerceptionUtils_Lidar(nh));

    percep_utils_lidar_ = std::make_shared<PerceptionUtils_Lidar>(nh);


    cloud_norm_.reset(new pcl::PointCloud<pcl::PointXYZI>());

    ros::Subscriber scan_sub_;


    scan_sub_ = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_origin", 10, scanCallback);

    pointcloud_norm_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/cloud_norm", 1);


    ROS_INFO("lidar_calibration_node start");
    ros::Duration(1.0).sleep();
    ros::spin();



    return 0;
}

void scanCallback(const sensor_msgs::PointCloud2::ConstPtr &scan){
    if(count == calibration_num){
        ROS_INFO("Writing calibration file");

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZI>(*cloud_norm_));

        pcl::toROSMsg(*cloud_norm_, output_msg);

        output_msg.header.stamp = ros::Time::now();
        output_msg.header.frame_id = "map";
        pointcloud_norm_pub_.publish(output_msg);

        ROS_INFO("count visible id");
        // percep_utils_lidar_->count_visible_id();
        
        percep_utils_lidar_->count_visible_id_with_intensity(*cloud_copy);

        ROS_INFO("read Detective Data");
        percep_utils_lidar_->readDetectiveData();

    }
    else if(count > calibration_num)
    {
        //no pose by default
        Eigen::Vector3d odom_pos;
        Eigen::Quaterniond odom_orient;
        odom_pos << 0, 0, 0;
        odom_orient = Eigen::Quaterniond::Identity();
        percep_utils_lidar_->setPose(odom_pos, odom_orient);

        percep_utils_lidar_->resetProjection();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_origin_(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::fromROSMsg(*scan, *cloud_origin_);

        for(int i = 0; i < cloud_origin_->points.size(); i++){
            percep_utils_lidar_->project_lidar_to_global(cloud_origin_->points[i].x, cloud_origin_->points[i].y, cloud_origin_->points[i].z);
        }
        
        percep_utils_lidar_->drawSphere();
        percep_utils_lidar_->drawProjection();

        ROS_WARN("Calibration end, please stop");

    }
    else{
        ROS_INFO("Pointcloud received, calibrating");
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_origin_(new pcl::PointCloud<pcl::PointXYZI>());

        pcl::fromROSMsg(*scan, *cloud_origin_);

        // for(int i = 0; i < cloud_origin_->points.size(); i++){
        //     percep_utils_lidar_->project_lidar_to_global(cloud_origin_->points[i].x, cloud_origin_->points[i].y, cloud_origin_->points[i].z);
        // }
        for(int i = 0; i < cloud_origin_->points.size(); i++){
            double dist = sqrt(cloud_origin_->points[i].x * cloud_origin_->points[i].x +
                                cloud_origin_->points[i].y * cloud_origin_->points[i].y +
                                cloud_origin_->points[i].z * cloud_origin_->points[i].z);
            pcl::PointXYZI point_norm;
            point_norm.x = cloud_origin_->points[i].x / dist * 15.0;
            point_norm.y = cloud_origin_->points[i].y / dist * 15.0;
            point_norm.z = cloud_origin_->points[i].z / dist * 15.0;
            point_norm.intensity = cloud_origin_->points[i].intensity;
            cloud_norm_->push_back(point_norm);
        }
    }
    count++;

}