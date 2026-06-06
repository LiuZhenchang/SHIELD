#include <lidar/perception_utils_lidar.h>
#include <random>

namespace shield
{

    PerceptionUtils_Lidar::PerceptionUtils_Lidar(ros::NodeHandle &nh)
    {

        nh.param("perception_utils_lidar/max_radius", max_radius_, 15.0);
        nh.param("perception_utils_lidar/min_radius", min_radius_, 1.0);
        nh.param("perception_utils_lidar/mount_yaw", mount_yaw_, 0.0);
        nh.param("perception_utils_lidar/mount_pitch", mount_pitch_, 0.0);
        nh.param("perception_utils_lidar/mount_roll", mount_roll_, 0.0);
        nh.param("perception_utils_lidar/theta_min", theta_min_, -7.0);
        nh.param("perception_utils_lidar/theta_max", theta_max_, 52.0);
        nh.param("perception_utils_lidar/phi_max", phi_max_, 180.0);
        nh.param("perception_utils_lidar/phi_min", phi_min_, -180.0);
        nh.param("perception_utils_lidar/theta_delta", d_theta_, 2.0);
        nh.param("perception_utils_lidar/phi_delta", d_phi_, 2.0);
        nh.param("perception_utils_lidar/lidar_mount_x", lidar_trans_.x(), 0.0);
        nh.param("perception_utils_lidar/lidar_mount_y", lidar_trans_.y(), 0.0);
        nh.param("perception_utils_lidar/lidar_mount_z", lidar_trans_.z(), 0.0);
        nh.param("perception_utils_lidar/search_radius", search_radius_, 6);
        nh.param("perception_utils_lidar/frame_id", frame_id_, string("world"));

        nh.param("perception_utils_lidar/read_calibration", read_calibration_, false);
        nh.param("perception_utils_lidar/cfg_dir", calibration_dir_, string("null"));
        nh.param("perception_utils_lidar/calibration_num_threshold", calibration_num_threshold_, 20); // 每个格子里落入多少点才认为这个方向回波有效
        nh.param("perception_utils_lidar/pos_threshold", pos_threshold_, 0.5); // 每个格子里落入多少点才认为这个方向回波有效


        // 此时还是度为单位
        Hsize_ = (int)((phi_max_ - phi_min_) / d_phi_);
        Vsize_ = std::ceil((theta_max_ - theta_min_) / d_theta_);

        drawer_ = nh.advertise<visualization_msgs::Marker>("/perception_utils_lidar/lidar_sphere", 1000);
        drawer1_ = nh.advertise<sensor_msgs::PointCloud2>("/perception_utils_lidar/projection", 1000);
        drawer2_ = nh.advertise<visualization_msgs::Marker>("/perception_utils_lidar/insideFOV", 1000);

        if (read_calibration_ == true)
        {
            ROS_ERROR("read calibration data");
            readDetectiveData();
        }
        else
        {
            ROS_ERROR("not read calibration data");
            // 此时还是度为单位
            Hsize_ = (int)((phi_max_ - phi_min_) / d_phi_);
            Vsize_ = std::ceil((theta_max_ - theta_min_) / d_theta_);
        }

        // 换算为弧度为单位

        mount_yaw_ *= M_PI / 180.0;
        mount_pitch_ *= M_PI / 180.0;
        mount_roll_ *= M_PI / 180.0;
        theta_min_ *= M_PI / 180.0;
        theta_max_ *= M_PI / 180.0;
        phi_max_ *= M_PI / 180.0;
        phi_min_ *= M_PI / 180.0;
        d_phi_ *= M_PI / 180.0;
        d_theta_ *= M_PI / 180.0;

        distance_.resize(Hsize_ * Vsize_);
        num_.resize(Hsize_ * Vsize_);
        pos_lidar_.resize(Hsize_ * Vsize_);

        SE3_matrix_lidar_.translation() = lidar_trans_;

        Eigen::Quaterniond q_lidar = Eigen::AngleAxisd(mount_yaw_, Eigen::Vector3d::UnitZ()) *
                                     Eigen::AngleAxisd(mount_pitch_, Eigen::Vector3d::UnitY()) *
                                     Eigen::AngleAxisd(mount_roll_, Eigen::Vector3d::UnitX());
        SE3_matrix_lidar_.linear() = q_lidar.matrix();
        SE3_matrix_lidar_.translation() = lidar_trans_;

        for (int id = 0; id < Hsize_ * Vsize_; id++)
        {
            double phi = (id % Hsize_) * d_phi_ + phi_min_ + 0.5 * d_phi_;
            double theta = std::floor(id / Hsize_) * d_theta_ + theta_min_ + 0.5 * d_theta_;
            Eigen::Vector3d pos;
            pos_lidar_[id].x() = max_radius_ * cos(theta) * cos(phi);
            pos_lidar_[id].y() = max_radius_ * cos(theta) * sin(phi);
            pos_lidar_[id].z() = max_radius_ * sin(theta);
        }
        ROS_INFO("PerceptionUtils_Lidar initialized");
        ROS_INFO("PerceptionUtils_Lidar Hsize = %d, Vsize = %d", Hsize_, Vsize_);
    }

    Eigen::Isometry3d PerceptionUtils_Lidar::getSE3MatrixL2B()
    {
        return SE3_matrix_lidar_;
    }

    void PerceptionUtils_Lidar::setPose(const Eigen::Vector3d &odom_pos, const Eigen::Quaterniond &odom_orient)
    {
        odom_pos_ = odom_pos;
        odom_orient_ = odom_orient;

        SE3_Matrix_odom_.linear() = odom_orient_.matrix();
        SE3_Matrix_odom_.translation() = odom_pos_;
    }

    void PerceptionUtils_Lidar::testinsideFOV()
    {
        // --- 生成 40x40x40 立方体内的随机测试点 ---
        const int num_points = 100;    // 随机点数量
        const double cube_size = 70.0; // 立方体边长
        const double half_size = cube_size / 2.0;

        // 随机数生成器初始化
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-half_size, half_size);
        std::uniform_real_distribution<> dis1(-half_size / 3.0, half_size);

        // 创建点标记
        visualization_msgs::Marker points_marker;
        points_marker.header.frame_id = frame_id_;
        points_marker.header.stamp = ros::Time::now();
        points_marker.ns = "fov_test_points";
        points_marker.id = 1;
        points_marker.type = visualization_msgs::Marker::POINTS;
        points_marker.scale.x = 0.3; // 点大小
        points_marker.scale.y = 0.3;
        points_marker.color.a = 1.0; // 透明度

        // 生成随机点并检查是否在FOV内
        for (int i = 0; i < num_points; ++i)
        {
            // 生成相对于飞机位置的随机偏移（-20 ~ +20 米）
            Eigen::Vector3d offset(dis(gen), dis(gen), dis1(gen));
            Eigen::Vector3d point_world = odom_pos_ + offset;

            // 判断点是否在FOV内
            bool is_inside = insideFOV(point_world);

            // 转换为Marker点
            geometry_msgs::Point p;
            p.x = point_world.x();
            p.y = point_world.y();
            p.z = point_world.z();
            points_marker.points.push_back(p);

            // 设置颜色（绿色=视野内，红色=视野外）
            std_msgs::ColorRGBA color;
            color.a = 0.8;
            if (is_inside)
            {
                points_marker.scale.x = 1.0; // 点大小
                points_marker.scale.y = 1.0;
                color.g = 1.0; // 绿色
            }
            else
            {
                color.r = 1.0; // 红色
            }
            points_marker.colors.push_back(color);
        }

        drawer2_.publish(points_marker); // 发布测试点
    }

    bool PerceptionUtils_Lidar::insideFOV(const Eigen::Vector3d &point_world)
    {
        // 将世界坐标系中的点转换到雷达局部坐标系
        Eigen::Vector3d relative_pos = point_world - odom_pos_;
        Eigen::Vector3d body_frame = odom_orient_.inverse() * relative_pos;
        Eigen::Vector3d lidar_frame = SE3_matrix_lidar_.inverse() * body_frame;

        // 计算半径
        double r = lidar_frame.norm();
        if (r < min_radius_ || r > max_radius_)
        {
            return false;
        }

        // 计算方位角phi（经度）和极角theta（纬度）
        double phi = atan2(lidar_frame.y(), lidar_frame.x()); // [-π, π]
        double theta = asin(lidar_frame.z() / r);             // [-π/2, π/2]

        // 检查极角theta范围
        if (theta < theta_min_ || theta > theta_max_)
        {
            return false;
        }

        // 检查方位角phi范围，处理phi范围跨越±π的情况
        if (phi_min_ <= phi_max_)
        {
            if (phi < phi_min_ || phi > phi_max_)
            {
                return false;
            }
        }
        else
        {
            // 当phi_min > phi_max时，检查phi是否在[phi_min, π]或[-π, phi_max]
            if (!(phi >= phi_min_ || phi <= phi_max_))
            {
                return false;
            }
        }

        return true;
    }

    // 激光雷达坐标系直角坐标转换为球面坐标
    void PerceptionUtils_Lidar::posToSphere(const Eigen::Vector3d &point, double &r, double &phi, double &theta)
    {
        r = sqrt(pow(point.x(), 2) + pow(point.y(), 2) + pow(point.z(), 2));
        phi = atan2(point.y(), point.x());
        theta = asin(point.z() / r);

    }

    int PerceptionUtils_Lidar::sphereToIndex(const double &r, const double &phi, const double &theta)
    {

        int v = std::floor((theta - theta_min_) / d_theta_);
        if (theta >= theta_max_ - epsilon)
        {
            v = Vsize_ - 1;
        }
        if (theta <= theta_min_ + epsilon)
        {
            v = 0;
        }

        // 计算经度索引
        int h = std::floor((phi - phi_min_) / d_phi_);
        if (phi >= phi_max_ - epsilon)
        {
            h = Hsize_ - 1;
        }
        if (phi <= phi_min_ + epsilon)
        {
            h = 0;
        }
        return v * Hsize_ + h;
    }

    void PerceptionUtils_Lidar::indexToSphere(const int &id, double &r, double &phi, double &theta)
    {
        int v = std::floor(id / Hsize_);
        theta = v * d_theta_ + theta_min_ + 0.5 * d_theta_;
        int h = std::floor(id % Hsize_);
        phi = h * d_phi_ + phi_min_ + 0.5 * d_phi_;
        r = distance_[id];
    }
    // 返回某个格子落入的点的global坐标，r即为平均的distance
    Eigen::Vector3d PerceptionUtils_Lidar::indexToGlobal(const int &id)
    {
        double theta, phi, r;
        int v = std::floor(id / Hsize_);
        theta = v * d_theta_ + theta_min_ + 0.5 * d_theta_;
        int h = std::floor(id % Hsize_);
        phi = h * d_phi_ + phi_min_ + 0.5 * d_phi_;
        r = distance_[id];
        Eigen::Vector3d pos(r * cos(theta) * cos(phi), r * cos(theta) * sin(phi), r * sin(theta));
        Eigen::Vector3d pos_global = SE3_Matrix_odom_ * SE3_matrix_lidar_ * pos;
        return pos_global;
    }

    //把某个激光雷达系下的点云投影到球面，并统计距离、落入球面格子数量
    void PerceptionUtils_Lidar::project_lidar_to_global(const double& x , const double& y , const double& z){

        double r, phi, theta;

        posToSphere(Eigen::Vector3d(x, y, z), r, phi, theta);
        int id = sphereToIndex(r, phi, theta);
    
        
        distance_[id] = (distance_[id] * num_[id] + r)/(num_[id] + 1);//对distance取加权平均值
        num_[id] += 1;


    
    }

    // 把某个激光雷达系下的点云投影到球面，并统计距离、落入球面格子数量
    void PerceptionUtils_Lidar::updateProjection(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double size)
    {
        resetProjection();
        for (const auto &point : cloud->points)
        {
            Eigen::Vector3d pt(point.x, point.y, point.z);
            double r, phi, theta;
            posToSphere(pt, r, phi, theta);
            double res_angle = fabs(atan(size / r));
            if (res_angle < d_theta_ && res_angle < d_phi_)
            {
                int id = sphereToIndex(r, phi, theta);
                distance_[id] = (distance_[id] * num_[id] + r) / (num_[id] + 1);
                num_[id] += 1;
            }
            else
            {
                for (double p = phi - res_angle / 2; p <= phi + res_angle / 2; p += d_phi_)
                {
                    for (double t = theta - res_angle / 2; t <= theta + res_angle / 2; t += d_theta_)
                    {
                        int id = sphereToIndex(r, p, t);
                        distance_[id] = (distance_[id] * num_[id] + r) / (num_[id] + 1);
                        num_[id] += 1;
                    }
                }
            }
        }
    }

    void PerceptionUtils_Lidar::resetProjection()
    {
        // 重置球面投影坐标为球面，并将distance设置为0，落入球面格子的num设置为0
        std::fill(num_.begin(), num_.end(), 0);
        std::fill(distance_.begin(), distance_.end(), 0.0);
    }

    void PerceptionUtils_Lidar::drawProjection()
    {
        pcl::PointXYZRGB pt;
        pcl::PointCloud<pcl::PointXYZRGB> cloud1;
        double s = 1.0;
        double v = 1.0;

        for (int i = 0; i < Hsize_ * Vsize_; i += 1)
        {
            if (IsIdDeactivated(i))
            {
                Eigen::Vector3d pos_global = SE3_Matrix_odom_ * SE3_matrix_lidar_ * pos_lidar_[i];
                pt.x = pos_global.x();
                pt.y = pos_global.y();
                pt.z = pos_global.z();

                pt.r = 255;
                pt.g = 255;
                pt.b = 255;
                cloud1.push_back(pt);
            }
            else
            {
                if (num_[i] <= 0)
                    continue;

                Eigen::Vector3d pos_global = SE3_Matrix_odom_ * SE3_matrix_lidar_ * pos_lidar_[i];
                // Eigen::Vector3d pos_global =  SE3_Matrix_odom_ * pos_lidar_[i];

                pt.x = pos_global.x();
                pt.y = pos_global.y();
                pt.z = pos_global.z();

                double dist = std::max(std::min(distance_[i], max_radius_), min_radius_);
                double h = dist / max_radius_ * 360;

                // 处理色相的边界：h=360等同于h=0，h<0时自动修正为正角度
                h = fmod(h, 360.0);
                if (h < 0)
                    h += 360.0;

                // HSV转RGB的核心公式（基于行业标准算法）
                double c = v * s;                                // 颜色浓度（Chroma）
                double x = c * (1 - abs(fmod(h / 60.0, 2) - 1)); // 过渡色系数
                double m = v - c;                                // 亮度补偿（确保明度正确）

                // 初始化RGB基础值（0~1范围）
                double r = 0.0, g = 0.0, b = 0.0;

                // 根据色相所在的6个区间，分配c、x、0到RGB通道
                if (h >= 0 && h < 60)
                {
                    r = c;
                    g = x;
                    b = 0;
                }
                else if (h >= 60 && h < 120)
                {
                    r = x;
                    g = c;
                    b = 0;
                }
                else if (h >= 120 && h < 180)
                {
                    r = 0;
                    g = c;
                    b = x;
                }
                else if (h >= 180 && h < 240)
                {
                    r = 0;
                    g = x;
                    b = c;
                }
                else if (h >= 240 && h < 300)
                {
                    r = x;
                    g = 0;
                    b = c;
                }
                else
                { // 300 <= h < 360
                    r = c;
                    g = 0;
                    b = x;
                }
                pt.r = static_cast<u_int8_t>(round((r + m) * 255));
                pt.g = static_cast<u_int8_t>(round((g + m) * 255));
                pt.b = static_cast<u_int8_t>(round((b + m) * 255));
                cloud1.push_back(pt);
            }
        }
        cloud1.width = cloud1.points.size();
        cloud1.height = 1;
        cloud1.is_dense = true;
        cloud1.header.frame_id = frame_id_;
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud1, cloud_msg);
        // 3. 发布完整的 MarkerArray
        drawer1_.publish(cloud_msg);
    }

    void PerceptionUtils_Lidar::drawSphere()
    {
        // odom_pos_ = odom_pos;
        // Clean old marker_
        marker_.action = visualization_msgs::Marker::DELETE;
        drawer_.publish(marker_);
        // 清空之前的点
        marker_.points.clear();

        marker_.header.frame_id = frame_id_;
        marker_.header.stamp = ros::Time::now();
        marker_.ns = "current_pose";
        marker_.id = 0;
        marker_.type = visualization_msgs::Marker::LINE_LIST;
        marker_.action = visualization_msgs::Marker::ADD;

        // 这里不能设置，会导致画出来的发生奇怪的偏移
        // marker_.pose.position.x = odom_pos_.x() + lidar_trans_.x();
        // marker_.pose.position.y = odom_pos_.y() + lidar_trans_.y();
        // marker_.pose.position.z = odom_pos_.z() + lidar_trans_.z();
        // marker_.pose.orientation.x = odom_orient.x();
        // marker_.pose.orientation.y = odom_orient.y();
        // marker_.pose.orientation.z = odom_orient.z();
        // marker_.pose.orientation.w = odom_orient.w();


        // 样式设置
        marker_.scale.x = 0.05;
        marker_.scale.y = 1.0;
        marker_.scale.z = 1.0;
        marker_.color.r = 0.0;
        marker_.color.g = 1.0;
        marker_.color.b = 1.0;
        marker_.color.a = 0.5;


        // 生成网格点
        for (int i_theta = 0; i_theta < Vsize_ + 1; ++i_theta)
        { // 维度方向分段数（纬度）
            double theta = theta_min_ + i_theta * d_theta_;

            // 纬线（固定theta，沿phi方向）
            for (int i_phi = 0; i_phi < Hsize_ + 1; ++i_phi)
            { // 方位角方向分段数（经度）
                double phi = phi_min_ + i_phi * d_phi_;

                // 当前点（局部坐标系）
                Eigen::Vector3d p_local;
                p_local.x() = max_radius_ * cos(theta) * cos(phi);
                p_local.y() = max_radius_ * cos(theta) * sin(phi);
                p_local.z() = max_radius_ * sin(theta);

                // 将局部坐标系的点转换到世界坐标系
                Eigen::Vector3d p_world = SE3_Matrix_odom_ * SE3_matrix_lidar_ * p_local;

                // 转换为 geometry_msgs::Point
                geometry_msgs::Point p;
                p.x = p_world.x();
                p.y = p_world.y();
                p.z = p_world.z();

                // 连接到下一个phi点（经线）
                if (i_phi < Hsize_)
                {
                    double next_phi = phi + d_phi_;
                    Eigen::Vector3d p_next_phi_local;
                    p_next_phi_local.x() = max_radius_ * cos(theta) * cos(next_phi);
                    p_next_phi_local.y() = max_radius_ * cos(theta) * sin(next_phi);
                    p_next_phi_local.z() = max_radius_ * sin(theta);

                    Eigen::Vector3d p_next_phi_world = SE3_Matrix_odom_ * SE3_matrix_lidar_ * p_next_phi_local;

                    geometry_msgs::Point p_next_phi;
                    p_next_phi.x = p_next_phi_world.x();
                    p_next_phi.y = p_next_phi_world.y();
                    p_next_phi.z = p_next_phi_world.z();

                    marker_.points.push_back(p);
                    marker_.points.push_back(p_next_phi);
                }

                // 连接到下一个theta点（纬线）
                if (i_theta < Vsize_)
                {
                    double next_theta = theta + d_theta_;
                    Eigen::Vector3d p_next_theta_local;
                    p_next_theta_local.x() = max_radius_ * cos(next_theta) * cos(phi);
                    p_next_theta_local.y() = max_radius_ * cos(next_theta) * sin(phi);
                    p_next_theta_local.z() = max_radius_ * sin(next_theta);

                    Eigen::Vector3d p_next_theta_world = SE3_Matrix_odom_ * SE3_matrix_lidar_ * p_next_theta_local;

                    geometry_msgs::Point p_next_theta;
                    p_next_theta.x = p_next_theta_world.x();
                    p_next_theta.y = p_next_theta_world.y();
                    p_next_theta.z = p_next_theta_world.z();

                    marker_.points.push_back(p);
                    marker_.points.push_back(p_next_theta);
                }
            }
        }
        drawer_.publish(marker_);
    }
    Eigen::Vector3d PerceptionUtils_Lidar::getGlobalPos(const int &id)
    {

        return SE3_Matrix_odom_ * SE3_matrix_lidar_ * pos_lidar_[id];
    }

    double PerceptionUtils_Lidar::getDistance(const int &id)
    {
        return distance_[id];
    }
    int PerceptionUtils_Lidar::getNumber(const int &id)
    {
        return num_[id];
    }
    int PerceptionUtils_Lidar::getMaxIndex()
    {
        return Hsize_ * Vsize_;
    }

    bool PerceptionUtils_Lidar::neighborNumber(const int &id)
    {
        int h = std::floor(id % Hsize_);
        int v = std::floor(id / Hsize_);
        for (int dh = -search_radius_; dh <= search_radius_; ++dh)
        {
            for (int dv = -search_radius_; dv <= search_radius_; ++dv)
            {
                int nh = h + dh;
                int nv = v + dv;
                if (nh < 0 || nh >= Hsize_ || nv < 0 || nv >= Vsize_)
                {
                    continue;
                }
                else
                {
                    int num = getNumber(nv * Hsize_ + nh);
                    if (num > 0)
                    {
                        return false; // 如果邻居有num>0.说明这个点大概率不是真正的free空间，返回false
                    }
                    else
                    {
                        continue;
                    }
                }
            }
        }
        return true;
    }

    void PerceptionUtils_Lidar::count_visible_id()
    {

        std::ofstream calibration_file(calibration_dir_ + "/calibration.txt");
        ROS_INFO("calibration_dir_: %s", calibration_dir_.c_str());
        if (!calibration_file.is_open())
        {
            // 错误处理：文件无法打开
            std::cerr << "Error: Unable to open calibration file at "
                      << calibration_dir_ << "/calibration.txt" << std::endl;
            return;
        }

        calibration_file << "Hsize = " << Hsize_ << std::endl;
        calibration_file << "Vsize = " << Vsize_ << std::endl;

        for (int i = 0; i < Hsize_ * Vsize_; i++)
        {
            if (num_[i] <= calibration_num_threshold_)
            {
                calibration_file << i << std::endl;
            }
        }

        calibration_file.close();
        ROS_INFO("Calibration file created successfully");
        
    }

    void PerceptionUtils_Lidar::count_visible_id_with_intensity(const pcl::PointCloud<pcl::PointXYZI>& cloud)
    {

        std::ofstream calibration_file1(calibration_dir_ + "/calibration.txt");
        ROS_INFO("calibration_dir_: %s", calibration_dir_.c_str());
        if (!calibration_file1.is_open())
        {
            // 错误处理：文件无法打开
            std::cerr << "Error: Unable to open calibration file at "
                      << calibration_dir_ << "/calibration.txt" << std::endl;
            return;
        }

        calibration_file1 << "Hsize = " << Hsize_ << std::endl;
        calibration_file1 << "Vsize = " << Vsize_ << std::endl;

        ROS_INFO("Hsize_: %d", Hsize_);
        ROS_INFO("Vsize_: %d", Vsize_);

        int point_num = cloud.points.size();

        vector<double> num_intensity;
        vector<int> num;
        vector<Eigen::Vector2d> average_pos_deviation_;
        int size = Hsize_ * Vsize_;
        num_intensity.resize(size, 0.0);
        num.resize(size, 0);
        average_pos_deviation_.resize(size, Eigen::Vector2d::Zero());

        ROS_INFO("point_num: %d", point_num);
        ROS_INFO("vector size: %d", size);
        ROS_INFO("vector size: %d", num_intensity.size());

        for (int i = 0; i < point_num; i++)
        {
            Eigen::Vector3d pt(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
            double r, phi, theta;
            posToSphere(pt, r, phi, theta);
            int id = sphereToIndex(r, phi, theta);


            num_intensity[id] += cloud.points[i].intensity / 256.0; // 归一化到0-1之间
            num[id] += 1;


            double phi_center = (id % Hsize_) * d_phi_ + phi_min_ + 0.5 * d_phi_;
            double theta_center = std::floor(id / Hsize_) * d_theta_ + theta_min_ + 0.5 * d_theta_;


            average_pos_deviation_[id].x() += (phi - phi_center);
            average_pos_deviation_[id].y() += (theta - theta_center);
            
        }

        for(int i = 0; i < size; i++)
        {
            if (num[i] > 0)
            {
                average_pos_deviation_[i].x() /= (double)num[i];
                average_pos_deviation_[i].y() /= (double)num[i];
            }

        }

        ROS_INFO("judge deactive id");

        for (int i = 0; i < Hsize_ * Vsize_; i++)
        {
            ROS_INFO("ID = %d, phi_dev = %f, theta_dev = %f", i, average_pos_deviation_[i].x()/M_PI*180.0, average_pos_deviation_[i].y()/M_PI*180.0);
            if (num_intensity[i] <= calibration_num_threshold_ || sqrt(average_pos_deviation_[i].x()*average_pos_deviation_[i].x() + average_pos_deviation_[i].y()*average_pos_deviation_[i].y()) > pos_threshold_ / 180.0 * M_PI)
            {
                calibration_file1 << i << std::endl;
            }
        }
        ROS_INFO("close file");
        
        calibration_file1.close();
        ROS_INFO("Calibration file created successfully");
    }

    void PerceptionUtils_Lidar::readDetectiveData()
    {
        ROS_INFO("read calibration file");
        std::ifstream file(calibration_dir_ + "/calibration.txt");
        if (!file.is_open())
        {
            // 文件打开失败，可以抛出异常或设置错误标志
            Hsize_ = (int)(phi_max_ - phi_min_) / d_phi_;
            Vsize_ = std::ceil((theta_max_ - theta_min_) / d_theta_);
            ROS_ERROR("Error: Unable to open calibration file at %s/calibration.txt", calibration_dir_.c_str());
            return;
        }

        std::string line;
        // 读取第一行 (Hsize)
        if (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string dummy;
            if (!(iss >> dummy >> dummy >> Hsize_))
            { // 跳过 "Hsize" 和 "="
                // 解析错误处理
                return;
            }
        }

        // 读取第二行 (Vsize)
        if (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string dummy;
            if (!(iss >> dummy >> dummy >> Vsize_))
            { // 跳过 "Vsize" 和 "="
                // 解析错误处理
                return;
            }
        }

        // 读取剩余行 (检测器ID)
        int id;
        while (std::getline(file, line))
        {
            if (line.empty())
                continue; // 跳过空行
            std::istringstream iss(line);
            if (iss >> id)
            {
                deactive_id_.push_back(id);
            }
        }

        for (int i = 0; i < deactive_id_.size(); i++)
        {
            ROS_WARN("deactive id = %d", deactive_id_[i]);
        }

        // 将 vector 转换为 unordered_set
        deactive_set_ = std::unordered_set<int>(deactive_id_.begin(), deactive_id_.end());
    }

    bool PerceptionUtils_Lidar::iscalibrate()
    {
        return read_calibration_;
    }

    bool PerceptionUtils_Lidar::IsIdDeactivated(int id)
    {
        return deactive_set_.find(id) != deactive_set_.end();
    }

}
