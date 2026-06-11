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
        nh.param("perception_utils_lidar/calibration_num_threshold", calibration_num_threshold_, 20); // Minimum number of points that must fall into a cell for the return in that direction to be considered valid
        nh.param("perception_utils_lidar/pos_threshold", pos_threshold_, 0.5); // Minimum number of points that must fall into a cell for the return in that direction to be considered valid


        // Values are still in degrees at this point
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
            // Values are still in degrees at this point
            Hsize_ = (int)((phi_max_ - phi_min_) / d_phi_);
            Vsize_ = std::ceil((theta_max_ - theta_min_) / d_theta_);
        }

        // Convert to radians

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
        // --- Generate random test points inside a 40x40x40 cube ---
        const int num_points = 100;    // Number of random points
        const double cube_size = 70.0; // Cube edge length
        const double half_size = cube_size / 2.0;

        // Initialize the random number generator
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-half_size, half_size);
        std::uniform_real_distribution<> dis1(-half_size / 3.0, half_size);

        // Create the point marker
        visualization_msgs::Marker points_marker;
        points_marker.header.frame_id = frame_id_;
        points_marker.header.stamp = ros::Time::now();
        points_marker.ns = "fov_test_points";
        points_marker.id = 1;
        points_marker.type = visualization_msgs::Marker::POINTS;
        points_marker.scale.x = 0.3; // Point size
        points_marker.scale.y = 0.3;
        points_marker.color.a = 1.0; // Opacity

        // Generate random points and check whether they are inside the FOV
        for (int i = 0; i < num_points; ++i)
        {
            // Generate a random offset relative to the aircraft position (-20 ~ +20 meters)
            Eigen::Vector3d offset(dis(gen), dis(gen), dis1(gen));
            Eigen::Vector3d point_world = odom_pos_ + offset;

            // Determine whether the point is inside the FOV
            bool is_inside = insideFOV(point_world);

            // Convert to a Marker point
            geometry_msgs::Point p;
            p.x = point_world.x();
            p.y = point_world.y();
            p.z = point_world.z();
            points_marker.points.push_back(p);

            // Set the color (green = inside FOV, red = outside FOV)
            std_msgs::ColorRGBA color;
            color.a = 0.8;
            if (is_inside)
            {
                points_marker.scale.x = 1.0; // Point size
                points_marker.scale.y = 1.0;
                color.g = 1.0; // Green
            }
            else
            {
                color.r = 1.0; // Red
            }
            points_marker.colors.push_back(color);
        }

        drawer2_.publish(points_marker); // Publish the test points
    }

    bool PerceptionUtils_Lidar::insideFOV(const Eigen::Vector3d &point_world)
    {
        // Transform the point from the world coordinate frame into the lidar local coordinate frame
        Eigen::Vector3d relative_pos = point_world - odom_pos_;
        Eigen::Vector3d body_frame = odom_orient_.inverse() * relative_pos;
        Eigen::Vector3d lidar_frame = SE3_matrix_lidar_.inverse() * body_frame;

        // Compute the radius
        double r = lidar_frame.norm();
        if (r < min_radius_ || r > max_radius_)
        {
            return false;
        }

        // Compute the azimuth angle phi (longitude) and polar angle theta (latitude)
        double phi = atan2(lidar_frame.y(), lidar_frame.x()); // [-pi, pi]
        double theta = asin(lidar_frame.z() / r);             // [-pi/2, pi/2]

        // Check the polar angle theta range
        if (theta < theta_min_ || theta > theta_max_)
        {
            return false;
        }

        // Check the azimuth angle phi range, handling the case where the phi range crosses +/-pi
        if (phi_min_ <= phi_max_)
        {
            if (phi < phi_min_ || phi > phi_max_)
            {
                return false;
            }
        }
        else
        {
            // When phi_min > phi_max, check whether phi is in [phi_min, pi] or [-pi, phi_max]
            if (!(phi >= phi_min_ || phi <= phi_max_))
            {
                return false;
            }
        }

        return true;
    }

    // Convert Cartesian coordinates in the lidar coordinate frame to spherical coordinates
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

        // Compute the longitude index
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
    // Returns the global coordinates of the points that fall into a given cell; r is the average distance
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

    // Project a point cloud in a lidar coordinate frame onto the sphere, and accumulate distances and counts of points falling into the spherical cells
    void PerceptionUtils_Lidar::project_lidar_to_global(const double& x , const double& y , const double& z){

        double r, phi, theta;

        posToSphere(Eigen::Vector3d(x, y, z), r, phi, theta);
        int id = sphereToIndex(r, phi, theta);


        distance_[id] = (distance_[id] * num_[id] + r)/(num_[id] + 1);// Take the weighted average of distance
        num_[id] += 1;



    }

    // Project a point cloud in a lidar coordinate frame onto the sphere, and accumulate distances and counts of points falling into the spherical cells
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
        // Reset the spherical projection coordinates to the sphere, set distance to 0, and set the count of points falling into the spherical cells to 0
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

                // Handle hue boundaries: h=360 is equivalent to h=0, and h<0 is automatically corrected to a positive angle
                h = fmod(h, 360.0);
                if (h < 0)
                    h += 360.0;

                // Core HSV-to-RGB formula (based on the standard industry algorithm)
                double c = v * s;                                // Color saturation (Chroma)
                double x = c * (1 - abs(fmod(h / 60.0, 2) - 1)); // Transition color coefficient
                double m = v - c;                                // Brightness compensation (ensures correct value/lightness)

                // Initialize the base RGB values (0~1 range)
                double r = 0.0, g = 0.0, b = 0.0;

                // Assign c, x, 0 to the RGB channels based on which of the 6 hue intervals applies
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
        // 3. Publish the complete MarkerArray
        drawer1_.publish(cloud_msg);
    }

    void PerceptionUtils_Lidar::drawSphere()
    {
        // odom_pos_ = odom_pos;
        // Clean old marker_
        marker_.action = visualization_msgs::Marker::DELETE;
        drawer_.publish(marker_);
        // Clear the previous points
        marker_.points.clear();

        marker_.header.frame_id = frame_id_;
        marker_.header.stamp = ros::Time::now();
        marker_.ns = "current_pose";
        marker_.id = 0;
        marker_.type = visualization_msgs::Marker::LINE_LIST;
        marker_.action = visualization_msgs::Marker::ADD;

        // This must not be set here, as it would cause a strange offset in the drawn output
        // marker_.pose.position.x = odom_pos_.x() + lidar_trans_.x();
        // marker_.pose.position.y = odom_pos_.y() + lidar_trans_.y();
        // marker_.pose.position.z = odom_pos_.z() + lidar_trans_.z();
        // marker_.pose.orientation.x = odom_orient.x();
        // marker_.pose.orientation.y = odom_orient.y();
        // marker_.pose.orientation.z = odom_orient.z();
        // marker_.pose.orientation.w = odom_orient.w();


        // Style settings
        marker_.scale.x = 0.05;
        marker_.scale.y = 1.0;
        marker_.scale.z = 1.0;
        marker_.color.r = 0.0;
        marker_.color.g = 1.0;
        marker_.color.b = 1.0;
        marker_.color.a = 0.5;


        // Generate the grid points
        for (int i_theta = 0; i_theta < Vsize_ + 1; ++i_theta)
        { // Number of segments in the latitude direction
            double theta = theta_min_ + i_theta * d_theta_;

            // Latitude lines (fixed theta, sweeping along phi)
            for (int i_phi = 0; i_phi < Hsize_ + 1; ++i_phi)
            { // Number of segments in the azimuth direction (longitude)
                double phi = phi_min_ + i_phi * d_phi_;

                // Current point (local coordinate frame)
                Eigen::Vector3d p_local;
                p_local.x() = max_radius_ * cos(theta) * cos(phi);
                p_local.y() = max_radius_ * cos(theta) * sin(phi);
                p_local.z() = max_radius_ * sin(theta);

                // Transform the point from the local coordinate frame to the world coordinate frame
                Eigen::Vector3d p_world = SE3_Matrix_odom_ * SE3_matrix_lidar_ * p_local;

                // Convert to geometry_msgs::Point
                geometry_msgs::Point p;
                p.x = p_world.x();
                p.y = p_world.y();
                p.z = p_world.z();

                // Connect to the next phi point (longitude line)
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

                // Connect to the next theta point (latitude line)
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
                        return false; // If a neighbor has num>0, this point is most likely not truly free space, so return false
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
            // Error handling: the file cannot be opened
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
            // Error handling: the file cannot be opened
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


            num_intensity[id] += cloud.points[i].intensity / 256.0; // Normalize to the range 0-1
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
            // Failed to open the file; can throw an exception or set an error flag
            Hsize_ = (int)(phi_max_ - phi_min_) / d_phi_;
            Vsize_ = std::ceil((theta_max_ - theta_min_) / d_theta_);
            ROS_ERROR("Error: Unable to open calibration file at %s/calibration.txt", calibration_dir_.c_str());
            return;
        }

        std::string line;
        // Read the first line (Hsize)
        if (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string dummy;
            if (!(iss >> dummy >> dummy >> Hsize_))
            { // Skip "Hsize" and "="
                // Parse error handling
                return;
            }
        }

        // Read the second line (Vsize)
        if (std::getline(file, line))
        {
            std::istringstream iss(line);
            std::string dummy;
            if (!(iss >> dummy >> dummy >> Vsize_))
            { // Skip "Vsize" and "="
                // Parse error handling
                return;
            }
        }

        // Read the remaining lines (detector IDs)
        int id;
        while (std::getline(file, line))
        {
            if (line.empty())
                continue; // Skip empty lines
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

        // Convert the vector to an unordered_set
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
