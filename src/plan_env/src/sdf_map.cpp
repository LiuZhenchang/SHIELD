#include "plan_env/sdf_map.h"
#include "plan_env/map_ros.h"
// #include "plan_env/multi_map_manager.h"
#include <plan_env/raycast.h>
#include <sys/time.h>

namespace shield
{
    SDFMap::SDFMap()
    {
    }

    SDFMap::~SDFMap()
    {
    }

    void SDFMap::initMap(ros::NodeHandle &nh)
    {
        mp_.reset(new MapParam);
        md_.reset(new MapData);

        // Params of map properties
        double x_size, y_size, z_size;
        double x_offset, y_offset;
        
        nh.param("sdf_map/resolution", mp_->resolution_, -1.0);
        nh.param("sdf_map/map_size_x", x_size, -1.0);
        nh.param("sdf_map/map_size_y", y_size, -1.0);
        nh.param("sdf_map/map_size_z", z_size, -1.0);
        nh.param("sdf_map/x_offset", x_offset, 0.0);
        nh.param("sdf_map/y_offset", y_offset, 0.0);
        nh.param("sdf_map/obstacles_inflation", mp_->obstacles_inflation_, -1.0);
        nh.param("sdf_map/local_bound_inflate", mp_->local_bound_inflate_, 1.0);
        nh.param("sdf_map/ground_height", mp_->ground_height_, -5.0);
        nh.param("sdf_map/default_dist", mp_->default_dist_, 5.0);
        nh.param("sdf_map/optimistic", mp_->optimistic_, true);
        nh.param("sdf_map/signed_dist", mp_->signed_dist_, false);

        mp_->local_bound_inflate_ = max(mp_->resolution_, mp_->local_bound_inflate_);
        mp_->resolution_inv_ = 1 / mp_->resolution_;
        mp_->map_origin_ = Eigen::Vector3d(-x_size / 2.0 + x_offset, -y_size / 2.0 + y_offset, mp_->ground_height_);
        mp_->map_size_ = Eigen::Vector3d(x_size, y_size, z_size);
        for (int i = 0; i < 3; ++i)
            mp_->map_voxel_num_(i) = ceil(mp_->map_size_(i) / mp_->resolution_);
        mp_->map_min_boundary_ = mp_->map_origin_;
        mp_->map_max_boundary_ = mp_->map_origin_ + mp_->map_size_;

        // Params of raycasting-based fusion
        nh.param("sdf_map/p_hit", mp_->p_hit_, 0.70);
        nh.param("sdf_map/p_miss", mp_->p_miss_, 0.35);
        nh.param("sdf_map/p_min", mp_->p_min_, 0.12);
        nh.param("sdf_map/p_max", mp_->p_max_, 0.97);
        nh.param("sdf_map/p_occ", mp_->p_occ_, 0.80);
        nh.param("sdf_map/max_ray_length", mp_->max_ray_length_, -0.1);
        nh.param("sdf_map/min_ray_length", mp_->min_ray_length_, -0.1);
        nh.param("sdf_map/virtual_ceil_height", mp_->virtual_ceil_height_, -0.1);
        nh.param("sdf_map/esdf_range_x", mp_->esdf_update_range_.x(), 10.0);
        nh.param("sdf_map/esdf_range_y", mp_->esdf_update_range_.y(), 10.0);
        nh.param("sdf_map/esdf_range_z", mp_->esdf_update_range_.z(), 5.0);
        nh.param("sdf_map/ignore_height", mp_->ignore_height_, -1.0);
        nh.param("sdf_map/normal_search_num", mp_->normal_search_num_, 10);

        nh.param("sdf_map/raycast_type", mp_->raycast_type_, 1);
        nh.param("sdf_map/raycast_with_fov", mp_->raycast_with_fov_, true);
        nh.param("sdf_map/raycast_num_threshold", raycast_num_threshold_, 50);
        nh.param("sdf_map/raycast_cos_threshold", raycast_cos_threshold_, 0.95);
        nh.param("sdf_map/raycast_cos_threshold2", raycast_cos_threshold2_, 0.95);
        nh.param("sdf_map/raycast_dist_threshold", raycast_dist_threshold_, 5.0);
        nh.param("sdf_map/pointcloud_buffer_length", pointcloud_buffer_length_, 5);

        nh.param("general/frame_id", frame_id_, string("map"));

        auto logit = [](const double &x)
        { return log(x / (1 - x)); };
        mp_->prob_hit_log_ = logit(mp_->p_hit_);
        mp_->prob_miss_log_ = logit(mp_->p_miss_);
        mp_->clamp_min_log_ = logit(mp_->p_min_);
        mp_->clamp_max_log_ = logit(mp_->p_max_);
        mp_->min_occupancy_log_ = logit(mp_->p_occ_);
        mp_->unknown_flag_ = 0.01;
        cout << "hit: " << mp_->prob_hit_log_ << ", miss: " << mp_->prob_miss_log_
             << ", min: " << mp_->clamp_min_log_ << ", max: " << mp_->clamp_max_log_
             << ", thresh: " << mp_->min_occupancy_log_ << endl;

        // Initialize data buffer of map
        int buffer_size = mp_->map_voxel_num_(0) * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2);
        md_->occupancy_buffer_ = vector<double>(buffer_size, mp_->clamp_min_log_ - mp_->unknown_flag_);
        md_->occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
        md_->min_observed_dist_ = vector<double>(buffer_size, 30.0);
        md_->max_observed_angle_ = vector<double>(buffer_size, 0.0);
        md_->distance_buffer_neg_ = vector<double>(buffer_size, mp_->default_dist_);
        md_->distance_buffer_ = vector<double>(buffer_size, mp_->default_dist_);
        md_->point_num_ = vector<int>(buffer_size, 0);
        md_->count_hit_and_miss_ = vector<short>(buffer_size, 0);
        md_->count_hit_ = vector<short>(buffer_size, 0);
        md_->count_miss_ = vector<short>(buffer_size, 0);
        md_->flag_rayend_ = vector<char>(buffer_size, -1);
        md_->flag_visited_ = vector<char>(buffer_size, -1);
        md_->quality_buffer_ = vector<double>(buffer_size, 0);
        md_->tmp_buffer1_ = vector<double>(buffer_size, 0);
        md_->tmp_buffer2_ = vector<double>(buffer_size, 0);
        md_->plane_normal_ = vector<Eigen::Vector3d>(buffer_size, Eigen::Vector3d(0, 0, 0));
        md_->raycast_num_ = 0;
        md_->reset_updated_box_ = true;

        // Try retriving bounding box of map, set box to map size if not specified
        vector<string> axis = {"x", "y", "z"};
        for (int i = 0; i < 3; ++i)
        {
            nh.param("sdf_map/box_min_" + axis[i], mp_->box_mind_[i], mp_->map_min_boundary_[i]);
            nh.param("sdf_map/box_max_" + axis[i], mp_->box_maxd_[i], mp_->map_max_boundary_[i]);
        }
        posToIndex(mp_->box_mind_, mp_->box_min_);
        posToIndex(mp_->box_maxd_, mp_->box_max_);

        caster_.reset(new RayCaster);
        caster_->setParams(mp_->resolution_, mp_->map_origin_);

        for (int i = 0; i < 3; ++i)
        {
            md_->all_min_[i] = 1000000;
            md_->all_max_[i] = -1000000;
        }


        Eigen::Vector3d left_bottom, right_top, left_top, right_bottom;
        left_bottom = mp_->box_mind_;
        right_top = mp_->box_maxd_;

        left_top[0] = left_bottom[0];
        left_top[1] = right_top[1];
        left_top[2] = left_bottom[2];
        right_bottom[0] = right_top[0];
        right_bottom[1] = left_bottom[1];
        right_bottom[2] = left_bottom[2];
        right_top[2] = left_bottom[2];

        mp_->vertices_ = {left_bottom, right_bottom, right_top, left_top};

        mp_->vmin_ = mp_->vmax_ = mp_->vertices_[0];
        for (int j = 1; j < mp_->vertices_.size(); ++j)
        {
            for (int k = 0; k < 2; ++k)
            {
                mp_->vmin_[k] = min(mp_->vmin_[k], mp_->vertices_[j][k]);
                mp_->vmax_[k] = max(mp_->vmax_[k], mp_->vertices_[j][k]);
            }
        }

        for (int j = 0; j < 4; ++j)
        {
            Eigen::Vector3d dir = (mp_->vertices_[(j + 1) % 4] - mp_->vertices_[j]).normalized();
            mp_->normals_.push_back(dir);
        }

        // flt added 显示法向量
        normal_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/sdf_map/voxel_normals", 10);

        percep_utils_lidar_ = std::make_shared<PerceptionUtils_Lidar>(nh);
        percep_utils_lidar_->resetProjection();

        // Initialize ROS wrapper
        mr_.reset(new MapROS);
        mr_->setMap(this);
        mr_->node_ = nh;
        mr_->init();
        mr_->map_initialized_ = true;


    }

    bool SDFMap::isboundary(const Eigen::Vector3d &pos)
    {
        Eigen::Array3d min_diff = pos.array() - mp_->box_mind_.array();
        Eigen::Array3d max_diff = mp_->box_maxd_.array() - pos.array();
        return (abs(min_diff) <= 2.0 * mp_->resolution_).any() || (abs(max_diff) <= 2.0 * mp_->resolution_).any();
    }
    void SDFMap::resetBuffer()
    {
        resetBuffer(mp_->map_min_boundary_, mp_->map_max_boundary_);
        md_->local_bound_min_ = Eigen::Vector3i::Zero();
        md_->local_bound_max_ = mp_->map_voxel_num_ - Eigen::Vector3i::Ones();
    }

    void SDFMap::resetBuffer(const Eigen::Vector3d &min_pos, const Eigen::Vector3d &max_pos)
    {
        Eigen::Vector3i min_id, max_id;
        posToIndex(min_pos, min_id);
        posToIndex(max_pos, max_id);
        boundIndex(min_id);
        boundIndex(max_id);

        for (int x = min_id(0); x <= max_id(0); ++x)
            for (int y = min_id(1); y <= max_id(1); ++y)
                for (int z = min_id(2); z <= max_id(2); ++z)
                {
                    md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
                    md_->distance_buffer_[toAddress(x, y, z)] = mp_->default_dist_;
                }
    }

    template <typename F_get_val, typename F_set_val>
    void SDFMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim)
    {
        int v[mp_->map_voxel_num_(dim)];
        double z[mp_->map_voxel_num_(dim) + 1];

        int k = start;
        v[start] = start;
        z[start] = -std::numeric_limits<double>::max();
        z[start + 1] = std::numeric_limits<double>::max();

        for (int q = start + 1; q <= end; q++)
        {
            k++;
            double s;

            do
            {
                k--;
                s = ((f_get_val(q) + q * q) - (f_get_val(v[k]) + v[k] * v[k])) / (2 * q - 2 * v[k]);
            } while (s <= z[k]);

            k++;

            v[k] = q;
            z[k] = s;
            z[k + 1] = std::numeric_limits<double>::max();
        }

        k = start;

        for (int q = start; q <= end; q++)
        {
            while (z[k + 1] < q)
                k++;
            double val = (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
            f_set_val(q, val);
        }
    }

    void SDFMap::updateESDF3d()
    {
        Eigen::Vector3i min_esdf1 = md_->local_bound_min_;
        Eigen::Vector3i max_esdf1 = md_->local_bound_max_;
        Eigen::Vector3d min_esdf2_pos = mr_->odom_pos_ + mp_->esdf_update_range_;
        Eigen::Vector3d max_esdf2_pos = mr_->odom_pos_ - mp_->esdf_update_range_;
        Eigen::Vector3i min_esdf2, max_esdf2;
        posToIndex(min_esdf2_pos, min_esdf2);
        posToIndex(max_esdf2_pos, max_esdf2);
        Eigen::Vector3i min_esdf = min_esdf1.array().min(min_esdf2.array());
        Eigen::Vector3i max_esdf = max_esdf1.array().max(max_esdf2.array());

        if (mp_->optimistic_)
        {
            for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
                for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
                {
                    fillESDF(
                        [&](int z)
                        {
                            return md_->occupancy_buffer_inflate_[toAddress(x, y, z)] == 1 ? 0 : std::numeric_limits<double>::max();
                        },
                        [&](int z, double val)
                        { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
                        max_esdf[2], 2);
                }
        }
        else
        {
            for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
                for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
                {
                    fillESDF(
                        [&](int z)
                        {
                            int adr = toAddress(x, y, z);
                            return (md_->occupancy_buffer_inflate_[adr] == 1 ||
                                    md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3)
                                       ? 0
                                       : std::numeric_limits<double>::max();
                        },
                        [&](int z, double val)
                        { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
                        max_esdf[2], 2);
                }
        }

        for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
            {
                fillESDF([&](int y)
                         { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
                         [&](int y, double val)
                         { md_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
                         max_esdf[1], 1);
            }
        for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
            for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
            {
                fillESDF([&](int x)
                         { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
                         [&](int x, double val)
                         {
                             md_->distance_buffer_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
                         },
                         min_esdf[0], max_esdf[0], 0);
            }

        if (mp_->signed_dist_)
        {
            // Compute negative distance
            for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
                for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
                {
                    fillESDF(
                        [&](int z)
                        {
                            return md_->occupancy_buffer_inflate_[toAddress(x, y, z)] == 0 ? 0 : std::numeric_limits<double>::max();
                        },
                        [&](int z, double val)
                        { md_->tmp_buffer1_[toAddress(x, y, z)] = val; }, min_esdf[2],
                        max_esdf[2], 2);
                }
            for (int x = min_esdf[0]; x <= max_esdf[0]; x++)
                for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
                {
                    fillESDF([&](int y)
                             { return md_->tmp_buffer1_[toAddress(x, y, z)]; },
                             [&](int y, double val)
                             { md_->tmp_buffer2_[toAddress(x, y, z)] = val; }, min_esdf[1],
                             max_esdf[1], 1);
                }
            for (int y = min_esdf[1]; y <= max_esdf[1]; y++)
                for (int z = min_esdf[2]; z <= max_esdf[2]; z++)
                {
                    fillESDF([&](int x)
                             { return md_->tmp_buffer2_[toAddress(x, y, z)]; },
                             [&](int x, double val)
                             {
                                 md_->distance_buffer_neg_[toAddress(x, y, z)] = mp_->resolution_ * std::sqrt(val);
                             },
                             min_esdf[0], max_esdf[0], 0);
                }
            // Merge negative distance with positive
            for (int x = min_esdf(0); x <= max_esdf(0); ++x)
                for (int y = min_esdf(1); y <= max_esdf(1); ++y)
                    for (int z = min_esdf(2); z <= max_esdf(2); ++z)
                    {
                        int idx = toAddress(x, y, z);
                        if (md_->distance_buffer_neg_[idx] > 0.0)
                            md_->distance_buffer_[idx] += (-md_->distance_buffer_neg_[idx] + mp_->resolution_);
                    }
        }
    }

    void SDFMap::setCacheOccupancy(const int &adr, const int &occ)
    {
        // check address
        if (adr < 0 || adr >= md_->count_hit_.size())
        {
            ROS_ERROR_STREAM("SDFMap: setCacheOccupancy: address out of range");
            return;
        }
        // Add to update list if first visited
        if (md_->count_hit_[adr] == 0 && md_->count_miss_[adr] == 0)
            md_->cache_voxel_.push(adr);

        if (occ == 0)
            md_->count_miss_[adr] = 1;
        else /* if (occ == 1) */
            md_->count_hit_[adr] += 1;
    }

    void SDFMap::inputPointCloud(const pcl::PointCloud<pcl::PointXYZ> &points, const Eigen::Vector3d &sensor_pos)
    {
        if (!isInMap(sensor_pos))
            return;
        struct timeval t1, t2, t3, t4, t5;
        gettimeofday(&t1, NULL);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_original(new pcl::PointCloud<pcl::PointXYZ>());
        *cloud_original = points;

        updatePointNumInVoxels(*cloud_original);

        gettimeofday(&t2, NULL);
        long seconds = t2.tv_sec - t1.tv_sec;
        long usec = t2.tv_usec - t1.tv_usec;
        double elapsed = seconds + usec / 1e6;

        updateMapWithPoints(*cloud_original, sensor_pos);
        updateMapWithFOV(sensor_pos);

        gettimeofday(&t3, NULL);
        seconds = t3.tv_sec - t2.tv_sec;
        usec = t3.tv_usec - t2.tv_usec;
        elapsed = seconds + usec / 1e6;

        updateVoxelState();

        gettimeofday(&t4, NULL);
        seconds = t4.tv_sec - t3.tv_sec;
        usec = t4.tv_usec - t3.tv_usec;
        elapsed = seconds + usec / 1e6;

        percep_utils_lidar_->drawSphere();
        percep_utils_lidar_->drawProjection();

        gettimeofday(&t5, NULL);
        seconds = t5.tv_sec - t4.tv_sec;
        usec = t5.tv_usec - t4.tv_usec;
        elapsed = seconds + usec / 1e6;

        seconds = t4.tv_sec - t1.tv_sec;
        usec = t4.tv_usec - t1.tv_usec;
        elapsed = seconds + usec / 1e6;
    }

    void SDFMap::updateMapWithPoints(const pcl::PointCloud<pcl::PointXYZ> &points, const Eigen::Vector3d &sensor_pos)
    {
        // initialize data
        int vox_adr;
        int point_num = points.size();
        Eigen::Vector3d pt_w;
        Eigen::Vector3i idx;
        md_->raycast_num_ += 1;

        Eigen::Vector3d update_min = sensor_pos;
        Eigen::Vector3d update_max = sensor_pos;
        if (md_->reset_updated_box_)
        {
            md_->update_min_ = sensor_pos;
            md_->update_max_ = sensor_pos;
            md_->reset_updated_box_ = false;
        }

        struct timeval t0, t1, t2, t3, t4;
        double elapse1 = 0;
        double elapse2 = 0;
        double elapse3 = 0;
        long seconds;
        long usec;

        // calculate pointcloud normals
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>());
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        calculateCloudNormals(cloud_merge_, tree, cloud_normals, sensor_pos);
        // tree_.setInputCloud(cloud_merge_);

        gettimeofday(&t0, NULL);
        // update voxel information with points
        for (int i = 0; i < point_num; ++i)
        {
            gettimeofday(&t1, NULL);
            auto &pt = points.points[i];
            pt_w << pt.x, pt.y, pt.z;
            int tmp_flag;

            // Set flag for projected point
            if (!isInMap(pt_w))
            {
                // Find closest point in map and set free
                pt_w = closetPointInMap(pt_w, sensor_pos);
                tmp_flag = determineVoxelFlag(pt_w, sensor_pos, false);
                if (tmp_flag == -1)
                    continue;
            }
            else
            {
                tmp_flag = determineVoxelFlag(pt_w, sensor_pos, true);
                if (tmp_flag == -1)
                    continue;
            }
            posToIndex(pt_w, idx);
            vox_adr = toAddress(idx);
            setCacheOccupancy(vox_adr, tmp_flag);

            for (int k = 0; k < 3; ++k)
            {
                update_min[k] = min(update_min[k], pt_w[k]);
                update_max[k] = max(update_max[k], pt_w[k]);
            }
            // Raycasting between camera center and point
            if (md_->flag_rayend_[vox_adr] == md_->raycast_num_)
                continue;
            else
                md_->flag_rayend_[vox_adr] = md_->raycast_num_;
            caster_->input(pt_w, sensor_pos);

            // caster_->nextId(idx);
            indexToPos(idx, pt_w);

            bool ray_well_observed = false;
            bool ray_better_observed = false;
            updateObserveInfo(tree, cloud_normals, pt_w, sensor_pos, ray_well_observed, ray_better_observed);

            gettimeofday(&t2, NULL);
            seconds = t2.tv_sec - t1.tv_sec;
            usec = t2.tv_usec - t1.tv_usec;
            elapse1 = elapse1 + seconds + usec / 1e6;

            while (caster_->nextId(idx))
            {
                if (getOccupancy(idx) == OCCUPIED)
                {
                    Eigen::Vector3d normal = getPlaneNormal(toAddress(idx)); // 获取穿过点的法向量
                    Eigen::Vector3d ray_dir = (sensor_pos - pt_w).normalized();
                    Eigen::Vector3d cur_pos;
                    indexToPos(idx, cur_pos);
                    bool well_observed = (md_->max_observed_angle_[toAddress(idx)] > raycast_cos_threshold_);
                    if (cur_pos(2) < mp_->ignore_height_)
                    {
                        continue;
                    }
                    switch (mp_->raycast_type_)
                    {
                    case 1:
                    {
                        if (md_->point_num_[toAddress(idx)] < raycast_num_threshold_ &&
                            fabs(ray_dir.dot(normal)) > raycast_cos_threshold2_ &&
                            !well_observed && ray_well_observed)
                        { // 如果历史点云不够多，且观测角度还可以，消除
                            setCacheOccupancy(toAddress(idx), 0);
                        }
                        break;
                    }
                    case 2:
                    {
                        if (!well_observed && ray_well_observed)
                        {
                            setCacheOccupancy(toAddress(idx), 0);
                        }
                        break;
                    }
                    }
                }
                else
                {
                    setCacheOccupancy(toAddress(idx), 0);
                }
            }
            gettimeofday(&t3, NULL);
            seconds = t3.tv_sec - t2.tv_sec;
            usec = t3.tv_usec - t2.tv_usec;
            elapse2 = elapse2 + seconds + usec / 1e6;
        }
        gettimeofday(&t4, NULL);
        seconds = t4.tv_sec - t0.tv_sec;
        usec = t4.tv_usec - t0.tv_usec;
        elapse3 = seconds + usec / 1e6;


        Eigen::Vector3d bound_inf(mp_->local_bound_inflate_, mp_->local_bound_inflate_, 0);
        posToIndex(update_max + bound_inf, md_->local_bound_max_);
        posToIndex(update_min - bound_inf, md_->local_bound_min_);
        boundIndex(md_->local_bound_min_);
        boundIndex(md_->local_bound_max_);
        mr_->local_updated_ = true;

        // Bounding box for subsequent updating and overall updateing
        for (int k = 0; k < 3; ++k)
        {
            md_->update_min_[k] = min(update_min[k], md_->update_min_[k]);
            md_->update_max_[k] = max(update_max[k], md_->update_max_[k]);
            md_->all_min_[k] = min(update_min[k], md_->all_min_[k]);
            md_->all_max_[k] = max(update_max[k], md_->all_max_[k]);
        }
    }

    int SDFMap::determineVoxelFlag(Eigen::Vector3d pt_pos, Eigen::Vector3d sensor_pos, bool pt_in_map)
    {
        int tmp_flag = 0;
        Eigen::Vector3i idx;
        double length = (pt_pos - sensor_pos).norm();

        if (length > mp_->max_ray_length_)
        {
            pt_pos = (pt_pos - sensor_pos) / length * mp_->max_ray_length_ + sensor_pos;
            posToIndex(pt_pos, idx);
            if (pt_pos[2] < 0.2 || !isInMap(pt_pos) || getOccupancy(idx) == OCCUPIED)
            {
                // 如果穿回来发现是占用的，就不更新，防止后凿洞出来
                tmp_flag = -1;
            }
            else
            {
                tmp_flag = 0;
            }
        }
        else if (length < mp_->min_ray_length_)
        {
            tmp_flag = -1;
        }
        else
        {
            if (pt_in_map)
            {
                tmp_flag = 1;
            }
        }
        return tmp_flag;
    }

    void SDFMap::updateMapWithFOV(const Eigen::Vector3d &sensor_pos)
    {
        int maxid = percep_utils_lidar_->getMaxIndex();
        Eigen::Vector3d pt_w;
        Eigen::Vector3i idx;
        int vox_adr;
        bool flag = true;
        vector<Eigen::Vector3i> idx_list;
        if (!mp_->raycast_with_fov_)
            return;
        if (!isInMap(sensor_pos))
            return;
        for (int i = 0; i < maxid; i++)
        {
            if (percep_utils_lidar_->IsIdDeactivated(i))
            { // 如果校准时候就没有回波，这个方向不要做raycast
                continue;
            }
            int num = percep_utils_lidar_->getNumber(i);
            if (num <= 0 && percep_utils_lidar_->neighborNumber(i))
            { // 没有点投影到球面，从最开始即为free一直raycast
                pt_w = percep_utils_lidar_->getGlobalPos(i);
                // Find closest point in map and set free
                if (!isInMap(pt_w))
                {
                    pt_w = closetPointInMap(pt_w, sensor_pos);
                }

                // Eigen::Vector3d ray_dir = pt_w - sensor_pos;
                caster_->input(sensor_pos, pt_w); // 放入raycast
                flag = true;
                idx_list.clear();
                while (caster_->nextId(idx))
                { // 从最开始即为free一直raycast
                    if (!isInMap(idx))
                    {
                        continue;
                    }
                    vox_adr = toAddress(idx);
                    idx_list.push_back(idx);
                    if (getOccupancy(idx) == OCCUPIED)
                    {
                        // 向外raycast可以消除occupy
                        //  Eigen::Vector3d normal = getPlaneNormal(toAddress(idx));//获取穿过点的法向量
                        //  if(md_->point_num_[toAddress(idx)] < raycast_num_threshold_ && fabs(ray_dir.dot(normal)) > raycast_cos_threshold2_){//如果历史点云不够多，且观测角度还可以，消除
                        //      setCacheOccupancy(vox_adr, 0);
                        //  }
                        //  else{
                        //      break;
                        //  }
                        flag = false;
                        break;
                    }
                }
                if (flag)
                {
                    for (int i = 0; i < idx_list.size(); i++)
                    {
                        setCacheOccupancy(toAddress(idx_list[i]), 0);
                    }
                }
            }
        }
    }

    void SDFMap::updateObserveInfo(const pcl::search::KdTree<pcl::PointXYZ>::Ptr &tree,
                                   const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals,
                                   const Eigen::Vector3d &pt_pos,
                                   const Eigen::Vector3d &sensor_pos,
                                   bool &well_observed,
                                   bool &better_observed)
    {
        Eigen::Vector3d plane_normal;
        Eigen::Vector3i idx;
        posToIndex(pt_pos, idx);
        pcl::Normal normal; // pcl库计算的法向量
        pcl::PointXYZ searchPoint;
        searchPoint.x = pt_pos.x();
        searchPoint.y = pt_pos.y();
        searchPoint.z = pt_pos.z();

        std::vector<int> indices(1);
        std::vector<float> distances(1);

        int voxel_idx;
        if (tree->nearestKSearch(searchPoint, 1, indices, distances) > 0)
        {
            voxel_idx = indices[0];
            normal = cloud_normals->points[voxel_idx];
            plane_normal << normal.normal_x, normal.normal_y, normal.normal_z;
            plane_normal = plane_normal.normalized();
        }
        Eigen::Vector3d ray_dir = (sensor_pos - pt_pos);
        ray_dir = ray_dir.normalized();
        double norm1 = plane_normal.norm();
        double cos_theta;
        if (norm1 > 1e-3)
        {
            cos_theta = ray_dir.dot(plane_normal);
            if (fabs(cos_theta) > md_->max_observed_angle_[toAddress(idx)])
            {
                setPlaneNormal(toAddress(idx), plane_normal);
                setObservedAngle(toAddress(idx), fabs(cos_theta));
                better_observed = true;
            }
            well_observed = (fabs(cos_theta) > raycast_cos_threshold_);
        }
    }

    void SDFMap::updatePointNumInVoxels(const pcl::PointCloud<pcl::PointXYZ> &points)
    {
        pointcloud_buffer_.push_back(points); // 存入点云到buffer
        if (pointcloud_buffer_.size() > pointcloud_buffer_length_)
        {
            pointcloud_buffer_.erase(pointcloud_buffer_.begin());
        }
        cloud_merge_.reset(new pcl::PointCloud<pcl::PointXYZ>()); // 融合后点云的指针
        for (int i = 0; i < pointcloud_buffer_.size(); i++)       // 融合点云
        {
            *cloud_merge_ += pointcloud_buffer_[i];
        }

        std::fill(md_->point_num_.begin(), md_->point_num_.end(), 0); // 先清空Voxel里面的点云数量统计

        for (auto point : *cloud_merge_)
        { // 统计点云落到Voxel里面的数量
            Eigen::Vector3d pt_w(point.x, point.y, point.z);
            if (!isInMap(pt_w))
            {
                continue;
            }

            Eigen::Vector3i idx;
            posToIndex(pt_w, idx);
            int vox_adr = toAddress(idx);
            if (vox_adr >= md_->point_num_.size())
            {
                ROS_ERROR_STREAM("pt_w: " << pt_w.x() << ", " << pt_w.y() << ", " << pt_w.z() << ", idx: " << idx.transpose());
                ROS_ERROR_STREAM("md_->point_num_ size = " << md_->point_num_.size() << ", vox_adr = " << vox_adr);
            }
            md_->point_num_[vox_adr] += 1;
        }
    }

    void SDFMap::calculateCloudNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                       pcl::search::KdTree<pcl::PointXYZ>::Ptr &tree,
                                       pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                       const Eigen::Vector3d &sensor_pos)
    {
        if (cloud->empty())
        {
            return;
        }
        // 创建基于邻域的法向估计类对象, 基于omp并行加速，需配置开启OpenMP
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        ne.setNumberOfThreads(6);
        ne.setInputCloud(cloud);
        ne.setViewPoint(sensor_pos.x(), sensor_pos.y(), sensor_pos.z());
        ne.setSearchMethod(tree);
        // ne.setRadiusSearch(getResolution());
        ne.setKSearch(mp_->normal_search_num_);
        ne.compute(*normals);
    }

    void SDFMap::updateVoxelState()
    {
        vector<uint32_t> new_voxel_ids_;
        while (!md_->cache_voxel_.empty())
        {
            int adr = md_->cache_voxel_.front();
            md_->cache_voxel_.pop();
            double log_odds_update =
                md_->count_hit_[adr] >= md_->count_miss_[adr] ? mp_->prob_hit_log_ : mp_->prob_miss_log_;
            md_->count_hit_[adr] = md_->count_miss_[adr] = 0;

            if (adr > md_->occupancy_buffer_.size())
            {
                ROS_ERROR_STREAM("occupancy_bufer size = " << md_->occupancy_buffer_.size() << ", adr = " << adr);
                continue;
            }

            if (md_->occupancy_buffer_[adr] < mp_->clamp_min_log_ - 1e-3)
            {
                md_->occupancy_buffer_[adr] = mp_->min_occupancy_log_;
                new_voxel_ids_.push_back(adr);
            }

            md_->occupancy_buffer_[adr] =
                std::min(std::max(md_->occupancy_buffer_[adr] + log_odds_update, mp_->clamp_min_log_),
                         mp_->clamp_max_log_);
        }
    }

    visualization_msgs::Marker SDFMap::createNormalMarker(
        const Eigen::Vector3d &position,
        const Eigen::Vector3d &normal,
        int marker_id)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id_; // 坐标系与地图一致
        marker.header.stamp = ros::Time::now();
        marker.ns = "voxel_normals"; // 命名空间
        marker.id = marker_id;       // 唯一ID
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0; // 无旋转
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;

        // 箭头起点（体素中心）和终点（沿法向量方向）
        geometry_msgs::Point start, end;
        start.x = position.x();
        start.y = position.y();
        start.z = position.z();
        end.x = position.x() + normal.x() * 2.0; // 缩放法向量长度
        end.y = position.y() + normal.y() * 2.0;
        end.z = position.z() + normal.z() * 2.0;

        marker.points.push_back(start);
        marker.points.push_back(end);

        // 箭头样式设置
        marker.scale.x = 0.1; // 箭头杆直径
        marker.scale.y = 0.2; // 箭头头直径
        marker.scale.z = 0.5; // 箭头头长度
        marker.color.r = 1.0; // 红色箭头
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;               // 不透明度
        marker.lifetime = ros::Duration(5); // 存活时间0.5秒

        return marker;
    }

    void SDFMap::setObservedDist(const int &adr, const double &dist)
    {
        if (md_->min_observed_dist_[adr] > dist ||
            md_->min_observed_dist_[adr] == 0.0)
            md_->min_observed_dist_[adr] = dist;
    }
    double SDFMap::getObservedDist(const int &adr)
    {
        return md_->min_observed_dist_[adr];
    }
    void SDFMap::setObservedAngle(const int &adr, const double &angle_cos)
    {
        if (md_->max_observed_angle_[adr] < angle_cos ||
            md_->max_observed_angle_[adr] == 0.0)
            md_->max_observed_angle_[adr] = fabs(angle_cos);
    }

    void SDFMap::setPlaneNormal(const int &adr, Eigen::Vector3d &plane_normal)
    {
        md_->plane_normal_[adr] = plane_normal;
    }
    double SDFMap::getObservedAngle(const int &adr)
    {
        return md_->max_observed_angle_[adr];
    }
    Eigen::Vector3d SDFMap::getPlaneNormal(const int &adr)
    {
        if(adr>= md_->plane_normal_.size() || adr < 0){
            ROS_ERROR("getPlaneNormal OUT BOUND");
        }
        return md_->plane_normal_[adr];
    }
    Eigen::Vector3d SDFMap::closetPointInMap(
        const Eigen::Vector3d &pt, const Eigen::Vector3d &camera_pt)
    {
        Eigen::Vector3d diff = pt - camera_pt;
        Eigen::Vector3d max_tc = mp_->map_max_boundary_ - camera_pt;
        Eigen::Vector3d min_tc = mp_->map_min_boundary_ - camera_pt;
        double min_t = 1000000;
        for (int i = 0; i < 3; ++i)
        {
            if (fabs(diff[i]) > 0)
            {
                double t1 = max_tc[i] / diff[i];
                if (t1 > 0 && t1 < min_t)
                    min_t = t1;
                double t2 = min_tc[i] / diff[i];
                if (t2 > 0 && t2 < min_t)
                    min_t = t2;
            }
        }
        return camera_pt + (min_t - 1e-3) * diff;
    }

    void SDFMap::clearAndInflateLocalMap()
    {
        int inf_step = ceil(mp_->obstacles_inflation_ / mp_->resolution_);
        vector<Eigen::Vector3i> inf_pts(pow(2 * inf_step + 1, 3));

        for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
            for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
                for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z)
                {
                    md_->occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
                }

        // inflate newest occpuied cells
        for (int x = md_->local_bound_min_(0); x <= md_->local_bound_max_(0); ++x)
            for (int y = md_->local_bound_min_(1); y <= md_->local_bound_max_(1); ++y)
                for (int z = md_->local_bound_min_(2); z <= md_->local_bound_max_(2); ++z)
                {
                    int id1 = toAddress(x, y, z);
                    if (md_->occupancy_buffer_[id1] > mp_->min_occupancy_log_)
                    {

                        for (int inf_x = -inf_step; inf_x <= inf_step; ++inf_x)
                            for (int inf_y = -inf_step; inf_y <= inf_step; ++inf_y)
                                for (int inf_z = -inf_step; inf_z <= inf_step; ++inf_z)
                                {
                                    Eigen::Vector3i inf_pt(x + inf_x, y + inf_y, z + inf_z);
                                    if (!isInMap(inf_pt))
                                        continue;
                                    int inf_adr = toAddress(inf_pt);
                                    md_->occupancy_buffer_inflate_[inf_adr] = 1;
                                }
                    }
                }
    }

    bool SDFMap::isInObstacle(const Eigen::Vector3i idx)
    {
        auto nbrs = sixNeighbors(idx);
        for (auto nbr : nbrs)
        {
            if (getOccupancy(nbr) != SDFMap::OCCUPIED)
                return false;
        }
        return true;
    }

    inline vector<Eigen::Vector3i>
    SDFMap::sixNeighbors(const Eigen::Vector3i &voxel)
    {
        vector<Eigen::Vector3i> neighbors(6);
        Eigen::Vector3i tmp;

        tmp = voxel - Eigen::Vector3i(1, 0, 0);
        neighbors[0] = tmp;
        tmp = voxel + Eigen::Vector3i(1, 0, 0);
        neighbors[1] = tmp;
        tmp = voxel - Eigen::Vector3i(0, 1, 0);
        neighbors[2] = tmp;
        tmp = voxel + Eigen::Vector3i(0, 1, 0);
        neighbors[3] = tmp;
        tmp = voxel - Eigen::Vector3i(0, 0, 1);
        neighbors[4] = tmp;
        tmp = voxel + Eigen::Vector3i(0, 0, 1);
        neighbors[5] = tmp;

        return neighbors;
    }

    double SDFMap::getResolution()
    {
        return mp_->resolution_;
    }

    void SDFMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
    {
        ori = mp_->map_origin_, size = mp_->map_size_;
    }

    void SDFMap::getBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax)
    {
        bmin = mp_->box_mind_;
        bmax = mp_->box_maxd_;
        // bmin = mp_->vmin_;
        // bmax = mp_->vmax_;
        // bmin[2] = mp_->box_mind_[2];
        // bmax[2] = mp_->box_maxd_[2];
    }

    int SDFMap::getVoxelNum()
    {
        return mp_->map_voxel_num_[0] * mp_->map_voxel_num_[1] * mp_->map_voxel_num_[2];
    }

    void SDFMap::getUpdatedBox(Eigen::Vector3d &bmin, Eigen::Vector3d &bmax, bool reset)
    {
        bmin = md_->update_min_;
        bmax = md_->update_max_;
        if (reset)
            md_->reset_updated_box_ = true;
    }

    double SDFMap::getDistWithGrad(const Eigen::Vector3d &pos, Eigen::Vector3d &grad)
    {
        if (!isInMap(pos))
        {
            grad.setZero();
            return 0;
        }

        /* trilinear interpolation */
        Eigen::Vector3d pos_m = pos - 0.5 * mp_->resolution_ * Eigen::Vector3d::Ones();
        Eigen::Vector3i idx;
        posToIndex(pos_m, idx);
        Eigen::Vector3d idx_pos, diff;
        indexToPos(idx, idx_pos);
        diff = (pos - idx_pos) * mp_->resolution_inv_;

        double values[2][2][2];
        for (int x = 0; x < 2; x++)
            for (int y = 0; y < 2; y++)
                for (int z = 0; z < 2; z++)
                {
                    Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
                    values[x][y][z] = getDistance(current_idx);
                }

        double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
        double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
        double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
        double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
        double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
        double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
        double dist = (1 - diff[2]) * v0 + diff[2] * v1;

        grad[2] = (v1 - v0) * mp_->resolution_inv_;
        grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * mp_->resolution_inv_;
        grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
        grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
        grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
        grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);
        grad[0] *= mp_->resolution_inv_;

        return dist;
    }

    /******************LZC added program******************/
    double SDFMap::getOccupyInfo()
    {
        int voxel_num = md_->occupancy_buffer_.size();
        if (voxel_num != getVoxelNum())
            ROS_INFO("Wrong voxel number!");
        int count = 0;
        for (int i = 0; i < voxel_num; i++)
        {
            if (md_->occupancy_buffer_[i] >= mp_->clamp_min_log_)
            {
                count = count + 1;
            }
        }
        double percent = double(count) / voxel_num * 100;
        return percent;
    }

    Eigen::Vector3d SDFMap::getMapFloor()
    {
        Eigen::Vector3d floor;
        floor.x() = 0;
        floor.y() = 0;
        floor.z() = mp_->map_min_boundary_.z();
        return floor;
    }

    Eigen::Vector3d SDFMap::getMapCeil()
    {
        Eigen::Vector3d ceil;
        ceil.x() = 0;
        ceil.y() = 0;
        ceil.z() = mp_->map_max_boundary_.z();
        return ceil;
    }

    void SDFMap::savePointCloud()
    {
        pcl::PointXYZ pt;
        pcl::PointCloud<pcl::PointXYZ> cloud1;

        Eigen::Vector3i min_idx, max_idx;
        posToIndex(md_->all_min_, min_idx);
        posToIndex(md_->all_max_, max_idx);

        boundIndex(min_idx);
        boundIndex(max_idx);

        for (int x = min_idx[0]; x <= max_idx[0]; ++x)
            for (int y = min_idx[1]; y <= max_idx[1]; ++y)
                for (int z = min_idx[2]; z <= max_idx[2]; ++z)
                {
                    if (md_->occupancy_buffer_[toAddress(x, y, z)] >
                        mp_->min_occupancy_log_)
                    {
                        Eigen::Vector3d pos;
                        indexToPos(Eigen::Vector3i(x, y, z), pos);
                        if (pos(2) > 100)
                            continue;
                        if (pos(2) < 0.3)
                            continue;
                        pt.x = pos(0);
                        pt.y = pos(1);
                        pt.z = pos(2);
                        cloud1.push_back(pt);
                    }
                }

        cloud1.width = cloud1.points.size();
        cloud1.height = 1;
        cloud1.is_dense = true;
        cloud1.header.frame_id = frame_id_;
        pcl::io::savePCDFileASCII("saved_cloud.pcd", cloud1);
    }

    /*****************************************************/
}