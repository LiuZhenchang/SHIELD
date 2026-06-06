/**
 * This file is part of Fast-Planner.
 *
 * Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
 * Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
 * for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * Fast-Planner is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Fast-Planner is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _PLANNING_VISUALIZATION_H_
#define _PLANNING_VISUALIZATION_H_

#include <ros/ros.h>
#include <vector>
#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <bspline/non_uniform_bspline.h>

using std::vector;
namespace shield
{
    class PlanningVisualization
    {
    private:
        enum TRAJECTORY_PLANNING_ID
        {
            GOAL = 1,
            STATE = 100,
            PATH = 200,
            BSPLINE = 300,
            BSPLINE_CTRL_PT = 400,
            POLY_TRAJ = 500
        };

        /* data */
        /* visib_pub is seperated from previous ones for different info */
        ros::NodeHandle node;
        ros::Publisher global_planner_pub_; // 0, global TSP path
        ros::Publisher local_planner_pub_;  // 1, loacal trajectory
        ros::Publisher state_pub_;          // 2, UAV flight state
        ros::Publisher frontier_pub_;       // 3, frontier normal
        ros::Publisher viewpoint_pub_;      // 4, viewpoints samples
        ros::Publisher fov_pub_;            // 5, visibility constraints
        ros::Publisher environment_pub_;    // 6, task environment
        ros::Publisher hgrid_pub_;          // 7, hgrid visualize
        ros::Publisher hgrid_path_pub_;          // 8, hgrid path visualize

        vector<ros::Publisher> marker_pubs_;

        ros::Publisher text_pub_;
        ros::Publisher path_pub_;
        ros::Publisher frontier_cloud_pub_;
        ros::Publisher viewpoint_pose_pub_;

        std::string frame_id_;

    public:
        PlanningVisualization(/* args */) {}
        ~PlanningVisualization() {}
        PlanningVisualization(ros::NodeHandle &nh);

        void fillBasicInfo(visualization_msgs::Marker &mk, const Eigen::Vector3d &scale,
                           const Eigen::Vector4d &color, const string &ns, const int &id, const int &shape);
        void fillGeometryInfo(visualization_msgs::Marker &mk, const vector<Eigen::Vector3d> &list);
        void fillGeometryInfo(visualization_msgs::Marker &mk, const vector<Eigen::Vector3d> &list1,
                              const vector<Eigen::Vector3d> &list2);

        void drawBox(const Eigen::Vector3d &center, const Eigen::Vector3d &scale,
                     const Eigen::Vector4d &color, const string &ns, const int &id, const int &pub_id);
        void drawText(const Eigen::Vector3d &pos, const string &text, const double &scale,
                      const Eigen::Vector4d &color, const string &ns, const int &id, const int &pub_id);

        void drawSpheres(const vector<Eigen::Vector3d> &list, const double &scale,
                         const Eigen::Vector4d &color, const string &ns, const int &id, const int &pub_id);
        void drawCubes(const vector<Eigen::Vector3d> &list, const double &scale,
                       const Eigen::Vector4d &color, const string &ns, const int &id, const int &pub_id);
        void drawArrows(const vector<Eigen::Vector3d> &list1, const vector<Eigen::Vector3d> &list2,
                        const Eigen::Vector3d &scale, const Eigen::Vector4d &color, const string &ns, const int &id,
                        const int &pub_id);
        void drawLines(const vector<Eigen::Vector3d> &list1, const vector<Eigen::Vector3d> &list2,
                       const double &scale, const Eigen::Vector4d &color, const string &ns, const int &id,
                       const int &pub_id);
        void drawLines(const vector<Eigen::Vector3d> &list, const double &scale,
                       const Eigen::Vector4d &color, const string &ns, const int &id, const int &pub_id);
        void drawTextArray( const vector<Eigen::Vector3d> &list, const vector<string> &text, 
                            const double &scale, const vector<Eigen::Vector4d> &color,
                            const string &ns, const int &id);

        void drawPoseArray(geometry_msgs::PoseArray &pose_array);
        void drawCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud);
        void drawPath(const vector<Eigen::Vector3d> &path);
        void drawHGridPath(const vector<Eigen::Vector3d> &path);

        // draw a bspline trajectory
        void drawBspline(NonUniformBspline &bspline, double size, const Eigen::Vector4d &color,
                         bool show_ctrl_pts = false, double size2 = 0.1,
                         const Eigen::Vector4d &color2 = Eigen::Vector4d(1, 1, 0, 1),
                         const string &ns = "", int id1 = 0, int id2 = 0);

        void drawYawTraj(NonUniformBspline &pos, NonUniformBspline &yaw, const double &dt, const string &ns);
        void drawYawPath(NonUniformBspline &pos, const vector<double> &yaw, const double &dt, const string &ns);
        void getCuboidEdge(vector<Eigen::Vector3d> &list1, vector<Eigen::Vector3d> &list2, const Eigen::Vector3d &origin, const Eigen::Vector3d &size);

        void displaySphereList(const vector<Eigen::Vector3d> &list, double resolution, const Eigen::Vector4d &color, int id, int pub_id = 0);
        void drawHgridBox(const vector<Eigen::Vector3d> &centers, const Eigen::Vector3d &resolution);

        typedef std::shared_ptr<PlanningVisualization> Ptr;
    };
} // namespace shield
#endif