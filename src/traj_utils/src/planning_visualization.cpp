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

#include <traj_utils/planning_visualization.h>

using std::cout;
using std::endl;
namespace shield
{
  PlanningVisualization::PlanningVisualization(ros::NodeHandle &nh)
  {
    node = nh;
    node.param("general/frame_id", frame_id_, string("map"));

    // Initialize marker publisher
    //0
    global_planner_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/global_planner", 20);
    marker_pubs_.push_back(global_planner_pub_);
    //1
    local_planner_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/local_planner", 20);
    marker_pubs_.push_back(local_planner_pub_);
    //2
    state_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/state", 20);
    marker_pubs_.push_back(state_pub_);
    //3
    frontier_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/frontier", 20);
    marker_pubs_.push_back(frontier_pub_);
    //4
    viewpoint_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/viewpoint", 20);
    marker_pubs_.push_back(viewpoint_pub_);
    //5
    fov_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/fov", 20);
    marker_pubs_.push_back(fov_pub_);
    //6
    environment_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/environment", 20);
    marker_pubs_.push_back(environment_pub_);
    //7
    hgrid_pub_ = node.advertise<visualization_msgs::Marker>("/planning_vis/hgrid", 20);
    marker_pubs_.push_back(hgrid_pub_);
    //8
    hgrid_path_pub_ = node.advertise<nav_msgs::Path>("/planning_vis/hgrid_path", 1000);
    marker_pubs_.push_back(hgrid_path_pub_);

    

    // Initialize other publisher
    text_pub_ = node.advertise<visualization_msgs::MarkerArray>("/planning_vis/text", 20);
    path_pub_ = node.advertise<nav_msgs::Path>("/planning_vis/path", 20);
    frontier_cloud_pub_ = node.advertise<sensor_msgs::PointCloud2>("/planning_vis/frontier_cloud", 1000);
    viewpoint_pose_pub_ = node.advertise<geometry_msgs::PoseArray>("/planning_vis/viewpoint_pose", 100);
  }

  void PlanningVisualization::fillBasicInfo(visualization_msgs::Marker &mk,
                                            const Eigen::Vector3d &scale, const Eigen::Vector4d &color, const string &ns, const int &id,
                                            const int &shape)
  {
    mk.header.frame_id = frame_id_;
    mk.header.stamp = ros::Time::now();
    mk.id = id;
    mk.ns = ns;
    mk.type = shape;

    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = scale[0];
    mk.scale.y = scale[1];
    mk.scale.z = scale[2];
  }

  void PlanningVisualization::fillGeometryInfo(visualization_msgs::Marker &mk,
                                               const vector<Eigen::Vector3d> &list)
  {
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      mk.points.push_back(pt);
    }
  }

  void PlanningVisualization::fillGeometryInfo(visualization_msgs::Marker &mk,
                                               const vector<Eigen::Vector3d> &list1,
                                               const vector<Eigen::Vector3d> &list2)
  {
    geometry_msgs::Point pt;
    for (int i = 0; i < int(list1.size()); ++i)
    {
      pt.x = list1[i](0);
      pt.y = list1[i](1);
      pt.z = list1[i](2);
      mk.points.push_back(pt);

      pt.x = list2[i](0);
      pt.y = list2[i](1);
      pt.z = list2[i](2);
      mk.points.push_back(pt);
    }
  }

  void PlanningVisualization::drawBox(const Eigen::Vector3d &center,
                                      const Eigen::Vector3d &scale,
                                      const Eigen::Vector4d &color,
                                      const string &ns,
                                      const int &id,
                                      const int &pub_id)
  {
    visualization_msgs::Marker mk;
    fillBasicInfo(mk, scale, color, ns, id, visualization_msgs::Marker::CUBE);
    mk.action = visualization_msgs::Marker::DELETE;
    marker_pubs_[pub_id].publish(mk);

    mk.pose.position.x = center[0];
    mk.pose.position.y = center[1];
    mk.pose.position.z = center[2];
    mk.action = visualization_msgs::Marker::ADD;

    marker_pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  void PlanningVisualization::drawText(const Eigen::Vector3d &pos,
                                       const string &text,
                                       const double &scale,
                                       const Eigen::Vector4d &color,
                                       const string &ns,
                                       const int &id,
                                       const int &pub_id)
  {
    visualization_msgs::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                  visualization_msgs::Marker::TEXT_VIEW_FACING);

    // clean old marker
    mk.action = visualization_msgs::Marker::DELETE;
    marker_pubs_[pub_id].publish(mk);

    // pub new marker
    mk.text = text;
    mk.pose.position.x = pos[0];
    mk.pose.position.y = pos[1];
    mk.pose.position.z = pos[2];
    mk.action = visualization_msgs::Marker::ADD;
    marker_pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  void PlanningVisualization::drawSpheres(const vector<Eigen::Vector3d> &list,
                                          const double &scale,
                                          const Eigen::Vector4d &color,
                                          const string &ns,
                                          const int &id,
                                          const int &pub_id)
  {
    visualization_msgs::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                  visualization_msgs::Marker::SPHERE_LIST);

    // clean old marker
    mk.action = visualization_msgs::Marker::DELETE;
    marker_pubs_[pub_id].publish(mk);

    if (list.size() == 0)
      return;

    // pub new marker
    fillGeometryInfo(mk, list);
    mk.action = visualization_msgs::Marker::ADD;
    marker_pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  void PlanningVisualization::displaySphereList(const vector<Eigen::Vector3d> &list, double resolution,
                                                const Eigen::Vector4d &color, int id, int pub_id)
  {
    visualization_msgs::Marker mk;
    mk.header.frame_id = frame_id_;
    mk.header.stamp = ros::Time::now();
    mk.type = visualization_msgs::Marker::SPHERE_LIST;
    mk.action = visualization_msgs::Marker::DELETE;
    mk.id = id;
    marker_pubs_[pub_id].publish(mk);

    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++)
    {
      pt.x = list[i](0);
      pt.y = list[i](1);
      pt.z = list[i](2);
      mk.points.push_back(pt);
    }
    marker_pubs_[pub_id].publish(mk);
    ros::Duration(0.001).sleep();
  }  
  void PlanningVisualization::drawHgridBox(const vector<Eigen::Vector3d> &centers, const Eigen::Vector3d &resolution)
  {
    vector<Eigen::Vector3d> list1;
    vector<Eigen::Vector3d> list2;
    vector<Eigen::Vector3d> list1_temp;
    vector<Eigen::Vector3d> list2_temp;
    for (int i = 0; i < centers.size(); ++i)
    {
      list1_temp.clear();
      list2_temp.clear();
      Eigen::Vector3d origin = centers[i] - 0.5 * resolution;
      getCuboidEdge(list1_temp, list2_temp, origin, resolution);
      list1.insert(list1.end(), list1_temp.begin(), list1_temp.end());
      list2.insert(list2.end(), list2_temp.begin(), list2_temp.end());
    }
    drawLines(list1, list2, 0.5, Eigen::Vector4d(1, 1, 1, 0.1), "hgrid", 11, 1);
  }

  void PlanningVisualization::drawHGridPath(const vector<Eigen::Vector3d> &path)
  {
    nav_msgs::Path uav_path_;
    uav_path_.header.stamp = ros::Time::now();
    uav_path_.header.frame_id = frame_id_;
        geometry_msgs::PoseStamped uav_pos;
    uav_pos.header.frame_id = frame_id_;
    uav_pos.pose.orientation.w = 1;
    uav_pos.pose.orientation.x = 0;
    uav_pos.pose.orientation.y = 0;
    uav_pos.pose.orientation.z = 0;
    for(Eigen::Vector3d pos:path)
    {
      uav_pos.pose.position.x = pos.x();
      uav_pos.pose.position.y = pos.y();
      uav_pos.pose.position.z = pos.z();
      uav_path_.poses.push_back(uav_pos);
    }
    marker_pubs_[8].publish(uav_path_);
  }


  void PlanningVisualization::drawCubes(const vector<Eigen::Vector3d> &list,
                                        const double &scale,
                                        const Eigen::Vector4d &color,
                                        const string &ns,
                                        const int &id,
                                        const int &pub_id)
  {
    visualization_msgs::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                  visualization_msgs::Marker::CUBE_LIST);

    // clean old marker
    mk.action = visualization_msgs::Marker::DELETE;
    marker_pubs_[pub_id].publish(mk);

    if (list.size() == 0)
      return;

    // pub new marker
    fillGeometryInfo(mk, list);
    mk.action = visualization_msgs::Marker::ADD;
    marker_pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  void PlanningVisualization::drawArrows(const vector<Eigen::Vector3d> &list1,
                                         const vector<Eigen::Vector3d> &list2,
                                         const Eigen::Vector3d &scale,
                                         const Eigen::Vector4d &color,
                                         const string &ns,
                                         const int &id,
                                         const int &pub_id)
  {
    visualization_msgs::Marker mk;
    fillBasicInfo(mk, scale, color, ns, id,
                  visualization_msgs::Marker::ARROW);

    // clean old marker
    mk.action = visualization_msgs::Marker::DELETE;
    marker_pubs_[pub_id].publish(mk);

    if (list1.size() == 0)
      return;

    // pub new marker
    fillGeometryInfo(mk, list1, list2);
    mk.action = visualization_msgs::Marker::ADD;
    marker_pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  void PlanningVisualization::drawLines(const vector<Eigen::Vector3d> &list1,
                                        const vector<Eigen::Vector3d> &list2,
                                        const double &scale,
                                        const Eigen::Vector4d &color,
                                        const string &ns,
                                        const int &id,
                                        const int &pub_id)
  {
    visualization_msgs::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                  visualization_msgs::Marker::LINE_LIST);

    // clean old marker
    mk.action = visualization_msgs::Marker::DELETE;
    marker_pubs_[pub_id].publish(mk);

    if (list1.size() == 0)
      return;

    // pub new marker
    fillGeometryInfo(mk, list1, list2);
    mk.action = visualization_msgs::Marker::ADD;
    marker_pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  void PlanningVisualization::drawLines(const vector<Eigen::Vector3d> &list,
                                        const double &scale,
                                        const Eigen::Vector4d &color,
                                        const string &ns,
                                        const int &id,
                                        const int &pub_id)
  {
    visualization_msgs::Marker mk;
    fillBasicInfo(mk, Eigen::Vector3d(scale, scale, scale), color, ns, id,
                  visualization_msgs::Marker::LINE_LIST);

    // clean old marker
    mk.action = visualization_msgs::Marker::DELETE;
    marker_pubs_[pub_id].publish(mk);

    if (list.size() == 0)
      return;

    // split the single list into two
    vector<Eigen::Vector3d> list1, list2;
    for (int i = 0; i < list.size() - 1; ++i)
    {
      list1.push_back(list[i]);
      list2.push_back(list[i + 1]);
    }

    // pub new marker
    fillGeometryInfo(mk, list1, list2);
    mk.action = visualization_msgs::Marker::ADD;
    marker_pubs_[pub_id].publish(mk);
    ros::Duration(0.0005).sleep();
  }

  

  void PlanningVisualization::drawTextArray(const vector<Eigen::Vector3d> &list,
                                            const vector<string> &text,
                                            const double &scale,
                                            const vector<Eigen::Vector4d> &color,
                                            const string &ns,
                                            const int &id)
  {
    // clean old marker
    visualization_msgs::MarkerArray text_marker_array;
    visualization_msgs::Marker text_marker;
    text_marker.action = visualization_msgs::Marker::DELETEALL;
    text_marker_array.markers.push_back(text_marker);
    text_pub_.publish(text_marker_array);

    // draw new marker
    if (list.size() != text.size() || color.size() != list.size())
    {
      ROS_ERROR("array size not equal");
      return;
    }
    text_marker_array.markers.clear();
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.header.frame_id = frame_id_;
    text_marker.header.stamp = ros::Time::now();
    text_marker.scale.z = scale;
    text_marker.ns = ns;

    for (int i = 0; i < list.size(); ++i)
    {
      text_marker.pose.position.x = list[i][0];
      text_marker.pose.position.y = list[i][1];
      text_marker.pose.position.z = list[i][2];
      text_marker.text = text[i];
      text_marker.color.r = color[i](0);
      text_marker.color.g = color[i](1);
      text_marker.color.b = color[i](2);
      text_marker.color.a = color[i](3);
      text_marker_array.markers.push_back(text_marker);
      text_marker.id = id * 1000 + i;
    }
    text_pub_.publish(text_marker_array);
  }

  void PlanningVisualization::drawBspline(NonUniformBspline &bspline,
                                          double size,
                                          const Eigen::Vector4d &color,
                                          bool show_ctrl_pts, double size2,
                                          const Eigen::Vector4d &color2,
                                          const string &ns, int id1, int id2)
  {
    if (bspline.getControlPoint().size() == 0)
      return;

    vector<Eigen::Vector3d> traj_pts;
    double tm, tmp;
    bspline.getTimeSpan(tm, tmp);

    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::Vector3d pt = bspline.evaluateDeBoor(t);
      traj_pts.push_back(pt);
    }
    drawSpheres(traj_pts, size, color, ns, BSPLINE + id1 % 100, 1);

    // draw the control point
    if (!show_ctrl_pts)
      return;

    Eigen::MatrixXd ctrl_pts = bspline.getControlPoint();
    vector<Eigen::Vector3d> ctp;

    for (int i = 0; i < int(ctrl_pts.rows()); ++i)
    {
      Eigen::Vector3d pt = ctrl_pts.row(i).transpose();
      ctp.push_back(pt);
    }
    drawSpheres(ctp, size2, color2, ns, BSPLINE_CTRL_PT + id2 % 100, 1);
  }

  void PlanningVisualization::drawYawTraj(NonUniformBspline &pos, NonUniformBspline &yaw,
                                          const double &dt, const string &ns)
  {
    double duration = pos.getTimeSum();
    vector<Eigen::Vector3d> pts1, pts2;

    for (double tc = 0.0; tc <= duration + 1e-3; tc += dt)
    {
      Eigen::Vector3d pc = pos.evaluateDeBoorT(tc);
      pc[2] += 0.15;
      double yc = yaw.evaluateDeBoorT(tc)[0];
      Eigen::Vector3d dir(cos(yc), sin(yc), 0);
      Eigen::Vector3d pdir = pc + 1.0 * dir;
      pts1.push_back(pc);
      pts2.push_back(pdir);
    }
    drawLines(pts1, pts2, 0.04, Eigen::Vector4d(1, 0.5, 0, 1), ns, 0, 5);
  }

  void PlanningVisualization::drawYawPath(NonUniformBspline &pos, const vector<double> &yaw,
                                          const double &dt, const string &ns)
  {
    vector<Eigen::Vector3d> pts1, pts2;

    for (int i = 0; i < yaw.size(); ++i)
    {
      Eigen::Vector3d pc = pos.evaluateDeBoorT(i * dt);
      pc[2] += 0.3;
      Eigen::Vector3d dir(cos(yaw[i]), sin(yaw[i]), 0);
      Eigen::Vector3d pdir = pc + 1.0 * dir;
      pts1.push_back(pc);
      pts2.push_back(pdir);
    }
    drawLines(pts1, pts2, 0.04, Eigen::Vector4d(1, 0, 1, 1), ns, 1, 5);
  }

  void PlanningVisualization::drawPoseArray(geometry_msgs::PoseArray &pose_array)
  {
    pose_array.header.frame_id = frame_id_;
    pose_array.header.stamp = ros::Time::now();
    viewpoint_pose_pub_.publish(pose_array);
  }

  void PlanningVisualization::drawCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
  {
    cloud->height = 1;
    cloud->is_dense = true;
    cloud->header.frame_id = frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    frontier_cloud_pub_.publish(cloud_msg);
  }

  void PlanningVisualization::drawPath(const vector<Eigen::Vector3d> &path)
  {
    nav_msgs::Path uav_path_;
    uav_path_.header.stamp = ros::Time::now();
    uav_path_.header.frame_id = frame_id_;
    geometry_msgs::PoseStamped uav_pos;
    uav_pos.header.frame_id = frame_id_;
    uav_pos.pose.orientation.w = 1;
    uav_pos.pose.orientation.x = 0;
    uav_pos.pose.orientation.y = 0;
    uav_pos.pose.orientation.z = 0;
    for (Eigen::Vector3d pos : path)
    {
      uav_pos.pose.position.x = pos.x();
      uav_pos.pose.position.y = pos.y();
      uav_pos.pose.position.z = pos.z();
      uav_path_.poses.push_back(uav_pos);
    }
    path_pub_.publish(uav_path_);
  }

  void PlanningVisualization::getCuboidEdge(vector<Eigen::Vector3d> &list1,
                                            vector<Eigen::Vector3d> &list2,
                                            const Eigen::Vector3d &origin,
                                            const Eigen::Vector3d &size)
  {
    list1.clear();
    list2.clear();
    // Edge 1
    list1.push_back(origin + size.cwiseProduct(Eigen::Vector3d(0, 0, 0)));
    list2.push_back(origin + size.cwiseProduct(Eigen::Vector3d(1, 0, 0)));
    // Edge 2
    list1.push_back(origin + size.cwiseProduct(Eigen::Vector3d(0, 0, 0)));
    list2.push_back(origin + size.cwiseProduct(Eigen::Vector3d(0, 1, 0)));
    // Edge 3
    list1.push_back(origin + size.cwiseProduct(Eigen::Vector3d(0, 0, 0)));
    list2.push_back(origin + size.cwiseProduct(Eigen::Vector3d(0, 0, 1)));
    // Edge 4
    list1.push_back(origin + size.cwiseProduct(Eigen::Vector3d(1, 0, 0)));
    list2.push_back(origin + size.cwiseProduct(Eigen::Vector3d(1, 1, 0)));
    // Edge 5
    list1.push_back(origin + size.cwiseProduct(Eigen::Vector3d(1, 0, 0)));
    list2.push_back(origin + size.cwiseProduct(Eigen::Vector3d(1, 0, 1)));
    // Edge 6
    list1.push_back(origin + size.cwiseProduct(Eigen::Vector3d(0, 1, 0)));
    list2.push_back(origin + size.cwiseProduct(Eigen::Vector3d(1, 1, 0)));
    // Edge 7
    list1.push_back(origin + size.cwiseProduct(Eigen::Vector3d(0, 1, 0)));
    list2.push_back(origin + size.cwiseProduct(Eigen::Vector3d(0, 1, 1)));
    // Edge 8
    list1.push_back(origin + size.cwiseProduct(Eigen::Vector3d(0, 0, 1)));
    list2.push_back(origin + size.cwiseProduct(Eigen::Vector3d(1, 0, 1)));
    // Edge 9
    list1.push_back(origin + size.cwiseProduct(Eigen::Vector3d(0, 0, 1)));
    list2.push_back(origin + size.cwiseProduct(Eigen::Vector3d(0, 1, 1)));
    // Edge 10
    list1.push_back(origin + size.cwiseProduct(Eigen::Vector3d(1, 0, 1)));
    list2.push_back(origin + size.cwiseProduct(Eigen::Vector3d(1, 1, 1)));
    // Edge 11
    list1.push_back(origin + size.cwiseProduct(Eigen::Vector3d(1, 1, 0)));
    list2.push_back(origin + size.cwiseProduct(Eigen::Vector3d(1, 1, 1)));
    // Edge 12
    list1.push_back(origin + size.cwiseProduct(Eigen::Vector3d(0, 1, 1)));
    list2.push_back(origin + size.cwiseProduct(Eigen::Vector3d(1, 1, 1)));
  }

  // PlanningVisualization::
} // namespace shield