#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>

#include <queue>
#include <ros/ros.h>
#include <tuple>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <lidar/perception_utils_lidar.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

class RayCaster;

namespace shield
{
    struct MapParam;
    struct MapData;
    class MapROS;
    class MultiMapManager;

    class SDFMap
    {
    public:
        SDFMap();
        ~SDFMap();

        enum OCCUPANCY
        {
            UNKNOWN,
            FREE,
            OCCUPIED
        };

        void initMap(ros::NodeHandle &nh);
        // Body-frame point cloud, and the SE3 matrix from lidar to global frame
        void inputPointCloud(const pcl::PointCloud<pcl::PointXYZ> &points, const Eigen::Vector3d &sensor_pos);

        void posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id);
        void indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos);
        void boundIndex(Eigen::Vector3i &id);
        int toAddress(const Eigen::Vector3i &id);
        int toAddress(const int &x, const int &y, const int &z);
        bool isInMap(const Eigen::Vector3d &pos);
        bool isInMap(const Eigen::Vector3i &idx);
        bool isInBox(const Eigen::Vector3i &id);
        bool isInBox(const Eigen::Vector3d &pos);
        bool isInObstacle(const Eigen::Vector3i idx);
        int getOccupancy(const Eigen::Vector3d &pos);
        int getOccupancy(const Eigen::Vector3i &id);
        void getUpdatedBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax, bool reset = false);
        int getInflateOccupancy(const Eigen::Vector3d &pos);
        int getInflateOccupancy(const Eigen::Vector3i &id);
        double getDistance(const Eigen::Vector3d &pos);
        double getDistance(const Eigen::Vector3i &id);
        double getDistWithGrad(const Eigen::Vector3d &pos, Eigen::Vector3d &grad);
        void updateESDF3d();
        void resetBuffer();
        void resetBuffer(const Eigen::Vector3d &min, const Eigen::Vector3d &max);

        void getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size);
        void getBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax);
        double getResolution();
        int getVoxelNum();

        shared_ptr<MultiMapManager> mm_;
        friend MultiMapManager;

        /******************LZC added program******************/
        double getOccupyInfo();
        Eigen::Vector3d getMapFloor();
        Eigen::Vector3d getMapCeil();
        void savePointCloud();
        /*****************************************************/

        // flt added
        visualization_msgs::Marker createNormalMarker(const Eigen::Vector3d &position, const Eigen::Vector3d &normal, int marker_id);
        ros::Publisher normal_marker_pub_; // Publisher declared inside the class
        int marker_id_ = 0;                // Used to generate unique Marker IDs
        double raycast_cos_threshold_;
        double raycast_cos_threshold2_;
        double raycast_dist_threshold_;

        shared_ptr<PerceptionUtils_Lidar> percep_utils_lidar_;
        vector<pcl::PointCloud<pcl::PointXYZ>> pointcloud_buffer_; // Buffer storing multiple point cloud frames
        int pointcloud_buffer_length_;                             // Number of point cloud frames to cache

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merge_; // Merged point cloud
        int raycast_num_threshold_;

        bool isboundary(const Eigen::Vector3d &pos);

        std::string frame_id_;

        void setObservedDist(const int &adr, const double &dist);
        void setObservedAngle(const int &adr, const double &angle);
        void setPlaneNormal(const int &adr, Eigen::Vector3d &plane_normal);
        int determineVoxelFlag(Eigen::Vector3d pt_pos, Eigen::Vector3d sensor_pos, bool pt_in_map);

        double getObservedDist(const int &adr);
        double getObservedAngle(const int &adr);
        Eigen::Vector3d getPlaneNormal(const int &adr);
        vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i &voxel);

    private:
        void clearAndInflateLocalMap();
        void setCacheOccupancy(const int &adr, const int &occ);
        Eigen::Vector3d closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt);

        void updatePointNumInVoxels(const pcl::PointCloud<pcl::PointXYZ> &points);
        void updateMapWithFOV(const Eigen::Vector3d &sensor_pos);
        void updateMapWithPoints(const pcl::PointCloud<pcl::PointXYZ> &points, const Eigen::Vector3d &sensor_pos);
        void updateObserveInfo(const pcl::search::KdTree<pcl::PointXYZ>::Ptr &tree,
                               const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals,
                               const Eigen::Vector3d &pt_pos,
                               const Eigen::Vector3d &sensor_pos,
                               bool &well_observed,
                               bool &better_observed);
        void calculateCloudNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                    pcl::search::KdTree<pcl::PointXYZ>::Ptr &tree,
                                    pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                    const Eigen::Vector3d &sensor_pos);
        void updateVoxelState();

        template <typename F_get_val, typename F_set_val>
        void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);

        unique_ptr<MapParam> mp_;
        unique_ptr<MapData> md_;
        unique_ptr<MapROS> mr_;
        unique_ptr<RayCaster> caster_;
        // pcl::search::KdTree<pcl::PointXYZ> tree_;

        friend MapROS;



    public:
        typedef std::shared_ptr<SDFMap> Ptr;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct MapParam
    {
        // map properties
        Eigen::Vector3d map_origin_, map_size_;
        Eigen::Vector3d map_min_boundary_, map_max_boundary_;
        Eigen::Vector3d esdf_update_range_;
        Eigen::Vector3i map_voxel_num_;
        double resolution_, resolution_inv_;
        double obstacles_inflation_;
        double virtual_ceil_height_, ground_height_;
        Eigen::Vector3i box_min_, box_max_;
        Eigen::Vector3d box_mind_, box_maxd_;
        double default_dist_;
        bool optimistic_, signed_dist_;
        // map fusion
        double p_hit_, p_miss_, p_min_, p_max_, p_occ_; // occupancy probability
        double prob_hit_log_, prob_miss_log_;
        double clamp_min_log_, clamp_max_log_;
        double min_occupancy_log_; // logit
        double max_ray_length_;
        double min_ray_length_; // LZC added
        double local_bound_inflate_;
        double unknown_flag_;
        double ignore_height_;



        // vertices, box and normals of map in current drone's frame
        Eigen::Vector3d vmin_, vmax_;
        vector<Eigen::Vector3d> vertices_;
        vector<Eigen::Vector3d> normals_;
        int normal_search_num_;

        // raycast setting
        int raycast_type_;
        bool raycast_with_fov_;
    };

    struct MapData
    {
        // main map data, occupancy of each voxel and Euclidean distance
        std::vector<double> occupancy_buffer_;
        std::vector<char> occupancy_buffer_inflate_;
        std::vector<double> min_observed_dist_;  // Observation distance
        std::vector<double> max_observed_angle_; // Observation angle, stores the cosine value; 0 corresponds to a grazing view, 1 to a frontal view
        std::vector<double> quality_buffer_;     // Observation quality computed from observation distance and angle
        std::vector<double> distance_buffer_neg_;
        std::vector<double> distance_buffer_;
        std::vector<int> point_num_; // Number of lidar points falling into the voxel
        std::vector<double> tmp_buffer1_;
        std::vector<double> tmp_buffer2_;
        std::vector<Eigen::Vector3d> plane_normal_; // Voxel normal vector
        // data for updating
        vector<short> count_hit_, count_miss_, count_hit_and_miss_;
        vector<char> flag_rayend_, flag_visited_;
        char raycast_num_;
        queue<int> cache_voxel_;
        Eigen::Vector3i local_bound_min_, local_bound_max_;
        Eigen::Vector3d update_min_, update_max_; // Box for recent update
        Eigen::Vector3d all_min_, all_max_;       // Box for overall update
        bool reset_updated_box_;

        // Swarm base coordinate transform
        map<int, Eigen::Vector4d> swarm_transform_;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    inline void SDFMap::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
    {
        for (int i = 0; i < 3; ++i)
            id(i) = floor((pos(i) - mp_->map_origin_(i)) * mp_->resolution_inv_);
    }

    inline void SDFMap::indexToPos(const Eigen::Vector3i &id, Eigen::Vector3d &pos)
    {
        for (int i = 0; i < 3; ++i)
            pos(i) = (id(i) + 0.5) * mp_->resolution_ + mp_->map_origin_(i);
    }

    inline void SDFMap::boundIndex(Eigen::Vector3i &id)
    {
        Eigen::Vector3i id1;
        id1(0) = max(min(id(0), mp_->map_voxel_num_(0) - 1), 0);
        id1(1) = max(min(id(1), mp_->map_voxel_num_(1) - 1), 0);
        id1(2) = max(min(id(2), mp_->map_voxel_num_(2) - 1), 0);
        id = id1;
    }

    inline int SDFMap::toAddress(const int &x, const int &y, const int &z)
    {
        int buffer_size = md_->occupancy_buffer_.size();
        int temp = x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z;
        if(temp >= buffer_size || temp < 0){
            ROS_ERROR("toAddress error: index out of bound!");
        }
        return temp;
    }

    inline int SDFMap::toAddress(const Eigen::Vector3i &id)
    {
        return toAddress(id[0], id[1], id[2]);
    }

    inline bool SDFMap::isInMap(const Eigen::Vector3d &pos)
    {
        if (pos(0) < mp_->map_min_boundary_(0) + 1e-4 || pos(1) < mp_->map_min_boundary_(1) + 1e-4 ||
            pos(2) < mp_->map_min_boundary_(2) + 1e-4)
            return false;
        if (pos(0) > mp_->map_max_boundary_(0) - 1e-4 || pos(1) > mp_->map_max_boundary_(1) - 1e-4 ||
            pos(2) > mp_->map_max_boundary_(2) - 1e-4)
            return false;
        return true;
    }

    inline bool SDFMap::isInMap(const Eigen::Vector3i &idx)
    {
        if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0)
            return false;
        if (idx(0) > mp_->map_voxel_num_(0) - 1 || idx(1) > mp_->map_voxel_num_(1) - 1 ||
            idx(2) > mp_->map_voxel_num_(2) - 1)
            return false;
        return true;
    }

    inline bool SDFMap::isInBox(const Eigen::Vector3i &id)
    {
        for (int i = 0; i < 3; ++i)
        {
            if (id[i] < mp_->box_min_[i] || id[i] >= mp_->box_max_[i])
            {
                return false;
            }
        }
        return true;
    }

    inline bool SDFMap::isInBox(const Eigen::Vector3d &pos)
    {
        for (int i = 0; i < 3; ++i)
        {
            if (pos[i] <= mp_->box_mind_[i] || pos[i] >= mp_->box_maxd_[i])
            {
                return false;
            }
        }
        return true;
    }

    inline int SDFMap::getOccupancy(const Eigen::Vector3i &id)
    {
        if (!isInMap(id))
            return -1;
        double occ = md_->occupancy_buffer_[toAddress(id)];
        if (occ < mp_->clamp_min_log_ - 1e-3)
            return UNKNOWN;
        if (occ > mp_->min_occupancy_log_)
            return OCCUPIED;
        return FREE;
    }

    inline int SDFMap::getOccupancy(const Eigen::Vector3d &pos)
    {
        Eigen::Vector3i id;
        posToIndex(pos, id);
        return getOccupancy(id);
    }

    inline int SDFMap::getInflateOccupancy(const Eigen::Vector3i &id)
    {
        if (!isInMap(id))
            return -1;
        return int(md_->occupancy_buffer_inflate_[toAddress(id)]);
    }

    inline int SDFMap::getInflateOccupancy(const Eigen::Vector3d &pos)
    {
        Eigen::Vector3i id;
        posToIndex(pos, id);
        return getInflateOccupancy(id);
    }

    inline double SDFMap::getDistance(const Eigen::Vector3i &id)
    {
        if (!isInMap(id))
            return -1;
        return md_->distance_buffer_[toAddress(id)];
    }

    inline double SDFMap::getDistance(const Eigen::Vector3d &pos)
    {
        Eigen::Vector3i id;
        posToIndex(pos, id);
        return getDistance(id);
    }
}

#endif