#include <active_perception/frontier_finder.h>
#include <lidar/perception_utils_lidar.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>

namespace shield
{

  FrontierFinder::FrontierFinder(const EDTEnvironment::Ptr &edt,
                                 ros::NodeHandle &nh)
  {
    this->edt_env_ = edt;
    int voxel_num = edt->sdf_map_->getVoxelNum();
    frontier_flag_ = vector<char>(voxel_num, 0);
    fill(frontier_flag_.begin(), frontier_flag_.end(), 0);

    nh.param("frontier_finder/threshold_max", threshold_max_, 0.7);
    nh.param("frontier_finder/threshold_min", threshold_min_, 0.2);
    nh.param("frontier_finder/well_observed", well_observed_, 0.7);
    nh.param("frontier_finder/min_quality_av", min_quality_av_, 0.5);
    nh.param("frontier_finder/dist_threshold", dist_threshold_, 5.0);
    nh.param("frontier_finder/search_step", search_step_, 2);

    nh.param("frontier/cluster_min", cluster_min_, 10);
    nh.param("frontier/cluster_size_xy", cluster_size_xy_, -1.0);
    nh.param("frontier/cluster_size_z", cluster_size_z_, -1.0);
    nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);
    nh.param("frontier/min_candidate_clearance", min_candidate_clearance_, -1.0);
    nh.param("frontier/candidate_dphi", candidate_dphi_, -1.0);
    nh.param("frontier/candidate_rmax", candidate_rmax_, -1.0);
    nh.param("frontier/candidate_rmin", candidate_rmin_, -1.0);
    nh.param("frontier/candidate_rnum", candidate_rnum_, -1);
    nh.param("frontier/candidate_hmax", candidate_hmax_, -1.0);
    nh.param("frontier/candidate_hmin", candidate_hmin_, -1.0);
    nh.param("frontier/candidate_hnum", candidate_hnum_, -1);
    nh.param("frontier/min_visib_percent", min_visib_percent_, 0.8);
    nh.param("frontier/min_visib_num", min_visib_num_, -1);
    nh.param("frontier/ignore_height", ignore_height_, 0.5);
    nh.param("frontier/ignore_height_up", ignore_height_up_, 50.0);

    nh.param("frontier/vm", vm_, -1.0);
    nh.param("frontier/am", am_, -1.0);
    nh.param("frontier/yd", yd_, -1.0);
    nh.param("frontier/ydd", ydd_, -1.0);
    nh.param("frontier/w_dir", w_dir_, -1.0);

    frontier_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/frontier", 10);
    debug_pts_1_ = nh.advertise<sensor_msgs::PointCloud2>("/frontier_debug1", 10);
    debug_pts_2_ = nh.advertise<sensor_msgs::PointCloud2>("/frontier_debug2", 10);

    astar_.reset(new Astar);
    astar_->init(nh, edt);

    resolution_ = edt_env_->sdf_map_->getResolution();
    Eigen::Vector3d origin, size;
    edt_env_->sdf_map_->getRegion(origin, size);

    raycaster_.reset(new RayCaster);
    raycaster_->setParams(resolution_, origin);
    percep_utils_lidar_ = std::make_shared<PerceptionUtils_Lidar>(nh);
    target_frt_id_ = -1;
    resetTargetFrontier();

  }

  FrontierFinder::~FrontierFinder() {}

  void FrontierFinder::searchFrontiers(Eigen::Vector3d cur_pos, double cur_yaw)
  {
    ROS_INFO("FrontierFinder:: Search frontiers");
    if (deactive_target_frontier_)
    {
      ROS_WARN("*************************** The Frontier Will Be Deactived *****************************");
    }
    tmp_frontiers_.clear();

    // Bounding box of updated region
    Vector3d update_min, update_max;
    edt_env_->sdf_map_->getUpdatedBox(update_min, update_max, true);
    double resolution = edt_env_->sdf_map_->getResolution();
    update_min -= Eigen::Vector3d(2.0 * resolution, 2.0 * resolution, 2.0 * resolution);
    update_max += Eigen::Vector3d(2.0 * resolution, 2.0 * resolution, 2.0 * resolution);

    // Removed changed frontiers in updated map
    auto resetFlag = [&](list<Frontier>::iterator &iter, list<Frontier> &frontiers, bool force_eliminate = false) { //& denotes pass-by-reference; iter and frontiers are formal parameters, and changes to iter and frontiers will affect the actual arguments
      Eigen::Vector3i idx;
      for (auto cell : iter->cells_)
      { // Loop over the cells in frontiers, convert them to idx using posToIndex, then set the value in frontier_flag_ to 0
        edt_env_->sdf_map_->posToIndex(cell, idx);
        // Record whether each voxel in the map is a frontier: 1 if it is a frontier, 0 or -1 if not; the container size equals the number of voxels
        if (!force_eliminate)
          frontier_flag_[toadr(idx)] = 0; // Dynamic frontier, allows regeneration
        else
        {
          if (iter->type_ == QUALITYFTR)
          {                                  // If it is a quality frontier, mark it as -1
            frontier_flag_[toadr(idx)] = -1; // Static frontier, regeneration not allowed
          }
          else
          {                                  // If it is a free and unknown frontier, mark it as -2
            frontier_flag_[toadr(idx)] = -2; // Static frontier, regeneration not allowed
          }
        }
      }
      iter = frontiers.erase(iter); // Container storing all frontier information (Frontier)
    };


    removed_ids_.clear();

    int rmv_idx = 0;

    for (auto iter = frontiers_.begin(); iter != frontiers_.end();)
    {

      bool haveoverlap = haveOverlap(iter->box_min_, iter->box_max_, update_min, update_max);


      bool isfrtchanged = isFrontierChanged(*iter, cur_pos, cur_yaw);


      if (haveoverlap && isfrtchanged)
      {

        resetFlag(iter, frontiers_, false);

        removed_ids_.push_back(rmv_idx);
      }
      // Add here: if it is the target frt and needs to be eliminated, then eliminate it
      else if (iter->is_target_ == true && deactive_target_frontier_ == true)
      {
        resetFlag(iter, frontiers_, deactive_target_frontier_);
        removed_ids_.push_back(rmv_idx);
        deactive_target_frontier_ = false;
      }
      // Accumulated 3 arrivals, force elimination directly
      else if ((cur_pos - iter->viewpoints_[0].pos_).norm() < 0.5 && fabs(iter->viewpoints_[0].yaw_ - cur_yaw) < 0.3)
      {
        reach_count_++;
        if (reach_count_ > 3)
        {
          resetFlag(iter, frontiers_, true);
          removed_ids_.push_back(rmv_idx);
          reach_count_ = 0;
        }
      }
      else
      {
        ++rmv_idx;
        ++iter;
      }
    }


    frts_num_after_remove_ = frontiers_.size();

    Vector3d search_min = update_min;
    Vector3d search_max = update_max;
    Vector3d box_min, box_max;

    edt_env_->sdf_map_->getBox(box_min, box_max);
    for (int k = 0; k < 3; ++k)
    {
      search_min[k] = max(search_min[k], box_min[k]);
      search_max[k] = min(search_max[k], box_max[k]);
    }
    // drawBox((search_min + search_max) / 2, search_max - search_min,
    //         Eigen::Vector4d(0, 0, 1, 0.3), "bbox", 0);
    Eigen::Vector3i min_id, max_id;
    edt_env_->sdf_map_->posToIndex(search_min, min_id);
    edt_env_->sdf_map_->posToIndex(search_max, max_id);

    Frontier frontier;
    // boundary_cells_.clear();


    for (int x = min_id(0); x <= max_id(0); x += search_step_)
      for (int y = min_id(1); y <= max_id(1); y += search_step_)
        for (int z = min_id(2); z <= max_id(2); z += search_step_)
        {
          // Scanning the updated region to find seeds of frontiers
          Eigen::Vector3i cur(x, y, z);
          Eigen::Vector3d pos;
          edt_env_->sdf_map_->indexToPos(cur, pos);

          if (pos[2] < ignore_height_ || pos[2] > ignore_height_up_)
            continue;
          if (!edt_env_->sdf_map_->isInMap(cur))
            continue;

          // Quality frontier
          if ((frontier_flag_[toadr(cur)] == 0 || frontier_flag_[toadr(cur)] == -2) &&
              (edt_env_->sdf_map_->getOccupancy(cur) == edt_env_->sdf_map_->OCCUPIED) && ((edt_env_->sdf_map_->getObservedAngle(toadr(cur)) < threshold_max_)))
          {
            expandFrontier(cur);
          }
          // free and unknown frontier
          else if ((frontier_flag_[toadr(cur)] == 0 || frontier_flag_[toadr(cur)] == -1) && knownfree(cur) && (isNeighborUnknown(cur) && isNeighborWellObserved(cur)))
          {
            expandFrontier(cur);
          }
        }

    splitLargeFrontiers(tmp_frontiers_);

    resetTargetFrontier();
    deactive_target_frontier_ = false;

  }

  void FrontierFinder::computeFrontiersToVisit(Eigen::Vector3d cur_pos)
  {
    int new_num = 0;         // Number of newly added valid frontiers
    first_new_frt_ = -1;
    first_new_ftr_ = frontiers_.end();
    // Try find viewpoints for each cluster and categorize them according to
    // viewpoint number
    for (auto &tmp_ftr : tmp_frontiers_)
    { // Iterate over all temporarily discovered frontiers
      // Search viewpoints around frontier
      sampleViewpoints(tmp_ftr);
      if (!tmp_ftr.viewpoints_.empty())
      {            // If valid viewpoints exist
        ++new_num; // Increment the count of valid frontiers

        list<Frontier>::iterator inserted =
            frontiers_.insert(frontiers_.end(), tmp_ftr); // Insert the current frontier at the end of the valid list and get the iterator to the insertion position
        // Sort the viewpoints by coverage fraction, best view in front
        if (tmp_ftr.type_ == QUALITYFTR)
        {
          // Define the viewpoint sorting rule (sorted in descending order by overall score)
          auto compare = [=](const Viewpoint &v1, const Viewpoint &v2)
          {
            double score_v1 = v1.quality_;
            double score_v2 = v2.quality_;
            return score_v1 > score_v2;
          };

          // auto compare_dist = [=](const Viewpoint &v1, const Viewpoint &v2) {
          //   double dist_v1 = v1.dist_;
          //   double dist_v2 = v2.dist_;
          //   return dist_v1 > dist_v2;
          // };

          // bool quality_check = false;
          // for(auto&vp : tmp_ftr.viewpoints_){
          //   if(vp.quality_ > min_quality_av_){
          //     quality_check = true;
          //     break;
          //   }
          // }

          // if(quality_check){
          //   // Sort the viewpoints (best viewpoint placed first)
          //   sort(inserted->viewpoints_.begin(), inserted->viewpoints_.end(), compare);
          // }
          // else{
          //   sort(inserted->viewpoints_.begin(), inserted->viewpoints_.end(), compare_dist);
          // }
          sort(inserted->viewpoints_.begin(), inserted->viewpoints_.end(), compare);
        }
        else if (tmp_ftr.type_ == FREEUNKNOWNFTR)
        {
          // Define the viewpoint sorting rule (sorted in descending order by overall score)
          auto compare_vis_num = [=](const Viewpoint &v1, const Viewpoint &v2)
          {
            double score_v1 = v1.visib_num_;
            double score_v2 = v2.visib_num_;
            return score_v1 > score_v2;
          };

          sort(inserted->viewpoints_.begin(), inserted->viewpoints_.end(), compare_vis_num);
        }

        // If it is the first valid frontier, record its position in the list
        if (!insert_frontier && !frontiers_.size() > 0)
        {
          first_new_frt_ = frontiers_.size() - 1;
          insert_frontier = true;
          first_new_ftr_ = frontiers_.begin();
        }
        if (first_new_ftr_ == frontiers_.end())
        {
          first_new_ftr_ = inserted;
        }

        //           << tmp_ftr.viewpoints_.begin()->pos_.x() << ", "
        //           << tmp_ftr.viewpoints_.begin()->pos_.y() << ", "
        //           << tmp_ftr.viewpoints_.begin()->pos_.z() << ". "<< std::endl;
      }
      else
      { // If there are no valid viewpoints, mark all cells of this frontier as inactive
        for (auto cell : tmp_ftr.cells_)
        {
          Eigen::Vector3i idx_;
          edt_env_->sdf_map_->posToIndex(cell, idx_);
          frontier_flag_[toadr(idx_)] = 0;
        }
        // // Find no viewpoint, move cluster to dormant vector
        // dormant_frontiers_.push_back(tmp_ftr);
        // ++new_dormant_num;
      }
    }
    // Reset indices of frontiers
    int idx = 0;
    for (auto &ft : frontiers_)
    {
      ft.id_ = idx++;
    }
  }

  void FrontierFinder::expandFrontier(
      const Eigen::Vector3i &first /* , const int& depth, const int& parent_id */)
  {
    // Data for clustering
    queue<Eigen::Vector3i> cell_queue;
    vector<Eigen::Vector3d> expanded;
    Vector3d pos;

    int ftr_type;
    if (knownfree(first))
    {
      ftr_type = FREEUNKNOWNFTR;
    }
    else
    {
      ftr_type = QUALITYFTR;
      // return;
    }

    edt_env_->sdf_map_->indexToPos(first, pos); // first is the id of the first input voxel meeting the frontier requirement; here it is converted to pos
    expanded.push_back(pos);                    // Append a new element to the end of the vector, at the position right after the current last element
    cell_queue.push(first);                     // Append x to the end of the queue
    frontier_flag_[toadr(first)] = 1;           // Mark as 1, recording that it has become a frontier

    int adr_first = edt_env_->sdf_map_->toAddress(first);
    Eigen::Vector3d normal_first = edt_env_->sdf_map_->getPlaneNormal(adr_first);

    auto addCell = [&](int adr, Eigen::Vector3i nbr, Eigen::Vector3d pos)
    {
      edt_env_->sdf_map_->indexToPos(nbr, pos);
      int adr1 = edt_env_->sdf_map_->toAddress(nbr);
      Eigen::Vector3d normal = edt_env_->sdf_map_->getPlaneNormal(adr1);
      if (normal.dot(normal_first) > 0.8)
      { // Only classify as the same frontier if approximately on the same plane
        expanded.push_back(pos);
        cell_queue.push(nbr);
        frontier_flag_[adr] = 1;
      }
    };

    auto addCell2 = [&](int adr, Eigen::Vector3i nbr, Eigen::Vector3d pos)
    {
      edt_env_->sdf_map_->indexToPos(nbr, pos);
      int adr1 = edt_env_->sdf_map_->toAddress(nbr);
      expanded.push_back(pos);
      cell_queue.push(nbr);
      frontier_flag_[adr] = 1;
    };

    Frontier frontier;

    if (ftr_type == QUALITYFTR)
    {
      // Search frontier cluster based on region growing (distance clustering)
      while (!cell_queue.empty())
      {
        auto cur = cell_queue.front();
        cell_queue.pop();
        auto nbrs = allNeighbors(cur);
        for (auto nbr : nbrs)
        {
          // Qualified cell should be inside bounding box and frontier cell not clustered
          int adr = toadr(nbr);
          Eigen::Vector3d pos1;
          edt_env_->sdf_map_->indexToPos(nbr, pos1);
          // 0 means it never became a frontier, 1 means it became a frontier, -1 means it was once a quality frt, -2 means it was once a free unknown frontier
          // 0 and -2 are allowed, the rest are not
          if ((frontier_flag_[adr] != 0 && frontier_flag_[adr] != -2) || !knownoccupy(nbr) || !edt_env_->sdf_map_->isInMap(nbr) || pos1[2] < ignore_height_ || pos1[2] > ignore_height_up_)
          { // frontier free that borders both occupy and unknown at the same time
            continue;
          }
          // if((knownfree(nbr) && (isNeighborOccupy(nbr)) && (isNeighborUnknown(nbr)))){//frontier free that borders both occupy and unknown at the same time
          if ((edt_env_->sdf_map_->getOccupancy(nbr) == edt_env_->sdf_map_->OCCUPIED) && (edt_env_->sdf_map_->getObservedAngle(toadr(nbr)) < threshold_max_))
          { // frontier with poor observation quality
            addCell2(adr, nbr, pos);
          }
        }
      }
      // frontier.cells_ = expanded;
      // expanded.clear();
    }
    else if (ftr_type == FREEUNKNOWNFTR)
    {
      while (!cell_queue.empty())
      {
        auto cur = cell_queue.front();
        cell_queue.pop();
        auto nbrs = allNeighbors(cur);
        for (auto nbr : nbrs)
        {
          // Qualified cell should be inside bounding box and frontier cell not clustered
          int adr = toadr(nbr);
          Eigen::Vector3d pos1;
          edt_env_->sdf_map_->indexToPos(nbr, pos1);
          // 0 means it never became a frontier, 1 means it became a frontier, -1 means it was once a quality frt, -2 means it was once a free unknown frontier
          // 0 and -1 are allowed, the rest are not
          if ((frontier_flag_[adr] != 0 && frontier_flag_[adr] != -1) || !knownfree(nbr) || !edt_env_->sdf_map_->isInMap(nbr) || pos1[2] < ignore_height_ || pos1[2] > ignore_height_up_)
          { // frontier free that borders both occupy and unknown at the same time
            continue;
          }
          if (knownfree(nbr) && (isNeighborUnknown(nbr) && isNeighborWellObserved(nbr)))
          { // free unknown boundary
            addCell2(adr, nbr, pos);
          }
        }
      }
      // frontier.cells_ = expanded;
      // expanded.clear();
    }

    if (expanded.size() > cluster_min_)
    {
      // Compute detailed info
      Frontier frontier;
      frontier.type_ = ftr_type;
      frontier.cells_ = expanded;
      expanded.clear();
      computeFrontierInfo(frontier);
      tmp_frontiers_.push_back(frontier);
    }
    else
    {
      // for (auto cell : expanded) {
      //     Eigen::Vector3i idx_;
      //     edt_env_->sdf_map_->posToIndex(cell, idx_);
      //     frontier_flag_[toadr(idx_)] = 0;
      // }
    }
    // frontier.type_ = ftr_type;
    // computeFrontierInfo(frontier);
    // tmp_frontiers_.push_back(frontier);
  }

  void FrontierFinder::splitLargeFrontiers(list<Frontier> &frontiers)
  {
    list<Frontier> splits, tmps;
    for (auto it = frontiers.begin(); it != frontiers.end(); ++it)
    {
      // Check if each frontier needs to be split horizontally
      if (splitIn3D(*it, splits))
      {
        tmps.insert(tmps.end(), splits.begin(), splits.end());
        splits.clear();
      }
      else
        tmps.push_back(*it);
    }
    frontiers = tmps;
  }

  bool FrontierFinder::splitIn3D(const Frontier &frontier,
                                 list<Frontier> &splits)
  {
    auto mean = frontier.average_;
    bool need_split = false;
    for (auto cell : frontier.cells_)
    {
      Eigen::Vector3d dist = cell - mean;
      if (sqrt(pow(dist.x(), 2) + pow(dist.y(), 2)) > cluster_size_xy_ || abs(dist.z()) > cluster_size_z_)
      {
        need_split = true;
        break;
      }
    }
    if (!need_split)
      return false;

    // Compute covariance matrix of cells
    Eigen::Matrix3d cov;
    cov.setZero();
    for (auto cell : frontier.cells_)
    {
      Eigen::Vector3d diff = cell - mean;
      cov += diff * diff.transpose();
    }
    cov /= double(frontier.cells_.size());

    // Find eigenvector corresponds to maximal eigenvalue
    Eigen::EigenSolver<Eigen::Matrix3d> es(cov);
    auto values = es.eigenvalues().real();
    auto vectors = es.eigenvectors().real();
    int max_idx;
    double max_eigenvalue = -1000000;
    for (int i = 0; i < values.rows(); ++i)
    {
      if (values[i] > max_eigenvalue)
      {
        max_idx = i;
        max_eigenvalue = values[i];
      }
    }
    Eigen::Vector3d first_pc = vectors.col(max_idx);

    // Split the frontier into two groups along the first PC
    Frontier ftr1, ftr2;
    ftr1.type_ = frontier.type_;
    ftr2.type_ = frontier.type_;
    for (auto cell : frontier.cells_)
    {
      if ((cell - mean).dot(first_pc) >= 0)
        ftr1.cells_.push_back(cell);
      else
        ftr2.cells_.push_back(cell);
    }
    computeFrontierInfo(ftr1);
    computeFrontierInfo(ftr2);

    // Recursive call to split frontier that is still too large
    list<Frontier> splits2;
    if (splitIn3D(ftr1, splits2))
    {
      splits.insert(splits.end(), splits2.begin(), splits2.end());
      splits2.clear();
    }
    else
    {
      splits.push_back(ftr1);
    }

    if (splitIn3D(ftr2, splits2))
      splits.insert(splits.end(), splits2.begin(), splits2.end());
    else
    {
      splits.push_back(ftr2);
    }

    return true;
  }

  void FrontierFinder::testmap()
  {
    pcl::PointXYZRGB pt;
    pcl::PointCloud<pcl::PointXYZRGB> cloud1;
    Vector3d box_min, box_max;
    edt_env_->sdf_map_->getBox(box_min, box_max);
    ROS_INFO("max: %f %f %f", box_max[0], box_max[1], box_max[2]);
    for (int x = box_min(0) /* + 1 */; x < box_max(0);
         ++x)
      for (int y = box_min(1) /* + 1 */; y < box_max(1);
           ++y)
        for (int z = box_min(2) /* + 1 */; z < box_max(2);
             ++z)
        {
          Eigen::Vector3d pos = {x, y, z};
          Eigen::Vector3i id;
          edt_env_->sdf_map_->posToIndex(pos, id);
          int state = edt_env_->sdf_map_->getOccupancy(id);
          ROS_INFO("state: %d", state);
          if (state == edt_env_->sdf_map_->OCCUPIED)
          {
            ROS_INFO("OCCUPIED");
            Eigen::Vector3d pos;
            edt_env_->sdf_map_->indexToPos(id, pos);
            pt.x = pos(0);
            pt.y = pos(1);
            pt.z = pos(2);
            cloud1.push_back(pt);
          }
        }
    cloud1.width = cloud1.points.size();
    cloud1.height = 1;
    cloud1.is_dense = true;
    cloud1.header.frame_id = "world";
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud1, cloud_msg);
    frontier_pub_.publish(cloud_msg);
  };

  void FrontierFinder::drawFrontier(Frontier &frontier)
  {
    pcl::PointXYZRGB pt;
    pcl::PointCloud<pcl::PointXYZRGB> cloud1;
    int count = 0;

    for (auto cell : frontier.cells_)
    {
      count++;
      if (count % 100 == 0)
      {
        pt.x = cell(0);
        pt.y = cell(1);
        pt.z = cell(2);
        pt.r = 255;
        pt.g = 0;
        pt.b = 0;
        cloud1.push_back(pt);
      }
    }

    cloud1.width = cloud1.points.size();
    cloud1.height = 1;
    cloud1.is_dense = true;
    cloud1.header.frame_id = "world";
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud1, cloud_msg);
    frontier_pub_.publish(cloud_msg);
  }

  void FrontierFinder::drawDebugInfo(const vector<Eigen::Vector3d> &pts, int pub_id)
  {
    pcl::PointXYZRGB pt;
    pcl::PointCloud<pcl::PointXYZRGB> cloud1;
    for (auto cell : pts)
    {
      pt.x = cell(0);
      pt.y = cell(1);
      pt.z = cell(2);
      pt.r = 255;
      pt.g = 0;
      pt.b = 0;
      cloud1.push_back(pt);
    }
    cloud1.width = cloud1.points.size();
    cloud1.height = 1;
    cloud1.is_dense = true;
    cloud1.header.frame_id = "world";
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud1, cloud_msg);
    if (pub_id == 1)
      debug_pts_1_.publish(cloud_msg);
    else if (pub_id == 2)
      debug_pts_2_.publish(cloud_msg);
  }

  void FrontierFinder::computeFrontierInfo(Frontier &ftr)
  {
    // Compute average position, average normal and bounding box of cluster
    ftr.average_.setZero();
    ftr.normal_.setZero();
    ftr.box_max_ = ftr.cells_.front();
    ftr.box_min_ = ftr.cells_.front();
    for (auto cell : ftr.cells_)
    {
      ftr.average_ += cell;
      for (int i = 0; i < 3; ++i)
      {
        ftr.box_min_[i] = min(ftr.box_min_[i], cell[i]);
        ftr.box_max_[i] = max(ftr.box_max_[i], cell[i]);
      }
      // Compute normal
      Eigen::Vector3i idx;
      edt_env_->sdf_map_->posToIndex(cell, idx);
      int adr = edt_env_->sdf_map_->toAddress(idx);
      ftr.normal_ += edt_env_->sdf_map_->getPlaneNormal(adr);
    }
    ftr.average_ /= double(ftr.cells_.size());
    ftr.normal_ /= double(ftr.cells_.size());
    ftr.normal_.normalize();

    // // Compute downsampled cluster
    // downsample(ftr.cells_, ftr.filtered_cells_);
  }

  inline vector<Eigen::Vector3i>
  FrontierFinder::sixNeighbors(const Eigen::Vector3i &voxel)
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

  inline vector<Eigen::Vector3i>
  FrontierFinder::tenNeighbors(const Eigen::Vector3i &voxel)
  {
    vector<Eigen::Vector3i> neighbors(10);
    Eigen::Vector3i tmp;
    int count = 0;

    for (int x = -1; x <= 1; ++x)
    {
      for (int y = -1; y <= 1; ++y)
      {
        if (x == 0 && y == 0)
          continue;
        tmp = voxel + Eigen::Vector3i(x, y, 0);
        neighbors[count++] = tmp;
      }
    }
    neighbors[count++] = tmp - Eigen::Vector3i(0, 0, 1);
    neighbors[count++] = tmp + Eigen::Vector3i(0, 0, 1);
    return neighbors;
  }

  inline vector<Eigen::Vector3i>
  FrontierFinder::allNeighbors(const Eigen::Vector3i &voxel)
  {
    vector<Eigen::Vector3i> neighbors;
    Eigen::Vector3i tmp;
    for (int x = -2; x <= 2; ++x)
      for (int y = -2; y <= 2; ++y)
        for (int z = -2; z <= 2; ++z)
        {
          if (x == 0 && y == 0 && z == 0)
            continue;
          tmp = voxel + Eigen::Vector3i(x, y, z);
          if (edt_env_->sdf_map_->isInMap(tmp))
          {
            neighbors.push_back(tmp);
          }
        }
    return neighbors;
  }

  inline vector<Eigen::Vector3i>
  FrontierFinder::allNeighbors1(const Eigen::Vector3i &voxel)
  {
    vector<Eigen::Vector3i> neighbors;
    Eigen::Vector3i tmp;
    for (int x = -1; x <= 1; ++x)
      for (int y = -1; y <= 1; ++y)
        for (int z = -1; z <= 1; ++z)
        {
          if (x == 0 && y == 0 && z == 0)
            continue;
          tmp = voxel + Eigen::Vector3i(x, y, z);
          if (edt_env_->sdf_map_->isInMap(tmp))
          {
            neighbors.push_back(tmp);
          }
        }
    return neighbors;
  }

  inline bool FrontierFinder::isNeighborUnknown(const Eigen::Vector3i &voxel)
  {
    // At least one neighbor is unknown
    auto nbrs = sixNeighbors(voxel);
    for (auto nbr : nbrs)
    {
      if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
        return true;
    }
    return false;
  }

  inline bool FrontierFinder::isNeighborUnknown1(const Eigen::Vector3i &voxel)
  {
    // At least one neighbor is unknown
    auto nbrs = allNeighbors1(voxel);
    for (auto nbr : nbrs)
    {
      if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::UNKNOWN)
        return true;
    }
    return false;
  }

  inline bool FrontierFinder::isNeighborOccupy(const Eigen::Vector3i &voxel)
  {
    // At least one neighbor is unknown
    auto nbrs = allNeighbors(voxel);
    for (auto nbr : nbrs)
    {
      if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::OCCUPIED)
        return true;
    }
    return false;
  }
  inline bool FrontierFinder::isNeighborFree(const Eigen::Vector3i &voxel)
  {
    // At least one neighbor is unknown
    auto nbrs = sixNeighbors(voxel);
    for (auto nbr : nbrs)
    {
      if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::FREE)
        return true;
    }
    return false;
  }

  inline bool FrontierFinder::isNeighborWellObserved(const Eigen::Vector3i &voxel)
  {
    // At least one neighbor is Well Observed
    auto nbrs = allNeighbors(voxel);
    for (auto nbr : nbrs)
    {
      if (edt_env_->sdf_map_->getOccupancy(nbr) == SDFMap::OCCUPIED && edt_env_->sdf_map_->getObservedAngle(toadr(nbr)) >= well_observed_)
        return true;
    }
    return false;
  }

  inline int FrontierFinder::toadr(const Eigen::Vector3i &idx)
  {
    return edt_env_->sdf_map_->toAddress(idx);
  }

  inline bool FrontierFinder::knownfree(const Eigen::Vector3i &idx)
  {
    return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::FREE;
  }
  inline bool FrontierFinder::knownoccupy(const Eigen::Vector3i &idx)
  {
    return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::OCCUPIED;
  }

  inline bool FrontierFinder::unknown(const Eigen::Vector3i &idx)
  {
    return edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN;
  }

  bool FrontierFinder::haveOverlap(const Vector3d &min1, const Vector3d &max1,
                                   const Vector3d &min2, const Vector3d &max2)
  {
    // Check if two box have overlap part
    Vector3d bmin, bmax;
    for (int i = 0; i < 3; ++i)
    {
      bmin[i] = max(min1[i], min2[i]);
      bmax[i] = min(max1[i], max2[i]);
      if (bmin[i] > bmax[i] + 1e-3)
        return false;
    }
    return true;
  }

  bool FrontierFinder::isFrontierChanged(const Frontier &ft, Eigen::Vector3d &cur_pos, double &cur_yaw)
  {
    int change_num = 0;
    int thresh = 0;
    int visib_num = 0;
    // if (ft.type_ == UNKNOWNFTR) {
    //   thresh = 0;
    // } else {
    //   thresh = ft.cells_.size() * 0.08;
    // }

    thresh = ft.cells_.size() * 0.2;

    // in condition the frontier has viewpoints
    if (ft.viewpoints_.size() > 0)
    {

      // determine whether the viewpoint is reachable
      if (edt_env_->sdf_map_->getInflateOccupancy(ft.viewpoints_[0].pos_) != 0)
      {
        // std::cout << "FrontierFinder:: frontier " << ft.id_ << ", Viewpoint is not reachable" << std::endl;
        return true;
      }

      // determine whether the frontier is visible
      double quality_av;
      visib_num = countVisibleCells(ft.viewpoints_[0].pos_, ft.viewpoints_[0].yaw_, ft.cells_, ft.type_, quality_av, ft.is_target_);

      if (visib_num < min_visib_num_)
      {
        // std::cout << "FrontierFinder:: frontier " << ft.id_ << ", Frontier (" << ft.average_.transpose() << ") is not visible from vp (" << ft.viewpoints_[0].pos_.transpose() << ")" << std::endl;
        return true;
      }
    }
    else
    {
      std::cout << "FrontierFinder:: frontier " << ft.id_ << ", No Viewpoint" << std::endl;
      return true;
    }

    // determine whether the frontier's condition has been cahnged
    if (ft.type_ == QUALITYFTR)
    {
      // Check observation quality fore each cell
      for (auto cell : ft.cells_)
      {
        Eigen::Vector3i idx;
        edt_env_->sdf_map_->posToIndex(cell, idx);
        double observe_angle = edt_env_->sdf_map_->getObservedAngle(toadr(idx));
        if (observe_angle > threshold_max_)
        {
          change_num++;
        }
        if (change_num > thresh)
        {
          // std::cout << "FrontierFinder:: frontier " << ft.id_ << " quality changed" << ", change_num = " << change_num << std::endl;
          return true;
        }
      }
    }
    else if (ft.type_ == FREEUNKNOWNFTR)
    {
      for (auto cell : ft.cells_)
      {
        Eigen::Vector3i idx;
        edt_env_->sdf_map_->posToIndex(cell, idx);
        if (!(knownfree(idx) && isNeighborUnknown(idx)))
        {
          change_num++;
        }
        if (change_num > thresh)
        {
          // std::cout << "FrontierFinder:: frontier " << ft.id_ << " unknown changed" << ", change_num = " << change_num << std::endl;
          return true;
        }
      }
    }

    return false;
  }

  void FrontierFinder::getFrontiers(vector<vector<Eigen::Vector3d>> &clusters)
  {
    clusters.clear();
    for (auto frontier : frontiers_)
    {
      clusters.push_back(frontier.cells_);
    }

    // clusters.push_back(frontier.filtered_cells_);
  }
  void FrontierFinder::mygetFrontiers(vector<Frontier> &clusters)
  {
    clusters.clear();
    for (auto frontier : frontiers_)
    {
      clusters.push_back(frontier);
    }
  }

  void FrontierFinder::getViewpoints(vector<vector<Viewpoint>> &clusters)
  {
    clusters.clear();
    for (auto frontier : frontiers_)
    {
      clusters.push_back(frontier.viewpoints_);
    }
  }

  void FrontierFinder::getTopViewpointsInfo(
      const Vector3d &cur_pos, vector<Eigen::Vector3d> &points, vector<double> &yaws,
      vector<Eigen::Vector3d> &averages)
  {
    points.clear();
    yaws.clear();
    averages.clear();
    for (auto frontier : frontiers_)
    {
      bool no_view = true;
      // for (auto view : frontier.viewpoints_) {
      //   // Retrieve the first viewpoint that is far enough and has highest coverage
      //   if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
      //   points.push_back(view.pos_);
      //   yaws.push_back(view.yaw_);
      //   averages.push_back(frontier.average_);
      //   no_view = false;
      //   break;
      // }
      points.push_back(frontier.viewpoints_[0].pos_);
      yaws.push_back(frontier.viewpoints_[0].yaw_);
      averages.push_back(frontier.average_);
      no_view = false;

      if (no_view)
      {
        // All viewpoints are very close, just use the first one (with highest coverage).
        auto view = frontier.viewpoints_.front();
        points.push_back(view.pos_);
        yaws.push_back(view.yaw_);
        averages.push_back(frontier.average_);
      }
    }
  }

  void FrontierFinder::getboundary(vector<Vector3d> &clusters)
  {
    clusters.clear();
    clusters.insert(clusters.end(), boundary_cells_.begin(), boundary_cells_.end());
  }
  // Sample viewpoints around frontier's average position, check coverage to the
  // frontier cells
  void FrontierFinder::sampleViewpoints(Frontier &frontier)
  {
    // Evaluate sample viewpoints on circles, find ones that cover most cells
    for (double rc = candidate_rmin_,
                dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_;
         rc <= candidate_rmax_ + 1e-3; rc += dr)
      for (double phi = -M_PI; phi < M_PI; phi += candidate_dphi_)
        for (double h = candidate_hmin_, dh = (candidate_hmax_ - candidate_hmin_) / candidate_hnum_; h <= candidate_hmax_; h += dh)
        {
          const Vector3d sample_pos =
              frontier.average_ + rc * Vector3d(cos(phi), sin(phi), 0) + Vector3d(0, 0, h);

          // Qualified viewpoint is in bounding box and in safe region
          if (!edt_env_->sdf_map_->isInBox(sample_pos) || // exit if not in box, or in inflation, or not free
              edt_env_->sdf_map_->getInflateOccupancy(sample_pos) == 1 ||
              edt_env_->sdf_map_->getOccupancy(sample_pos) != SDFMap::FREE)
          {
            continue;
          }

          // Compute average yaw
          auto &cells = frontier.cells_;
          Eigen::Vector3d ref_dir = (frontier.average_ - sample_pos).normalized();

          double avg_yaw = 0.0;
          for (int i = 1; i < cells.size(); ++i)
          {
            Eigen::Vector3d dir = (cells[i] - sample_pos).normalized();
            double yaw = acos(dir.dot(ref_dir));
            if (ref_dir.cross(dir)[2] < 0)
              yaw = -yaw;
            avg_yaw += yaw;
          }
          avg_yaw = avg_yaw / cells.size() + atan2(ref_dir[1], ref_dir[0]);
          wrapYaw(avg_yaw);

          Eigen::Vector3i idx; // Used for raycast numbering
          double quality_av = 0;
          int visib_num = 0;
          int visib_num1 = 0;
          int visib_num2 = 0;
          visib_num1 = countVisibleCells(sample_pos, avg_yaw, cells, frontier.type_, quality_av, false);
          visib_num2 = countVisibleCells(sample_pos, avg_yaw + M_PI, cells, frontier.type_, quality_av, false);
          visib_num = visib_num1;
          if (visib_num2 > visib_num)
          {
            visib_num = visib_num2;
            avg_yaw = avg_yaw + M_PI;
            wrapYaw(avg_yaw);
          }

          if (frontier.type_ == QUALITYFTR)
          {
            // Compute the fraction of covered and visible cells
            if (visib_num > min_visib_num_ && !isnan(quality_av))
            {
              Viewpoint vp = {sample_pos, avg_yaw, visib_num, quality_av};
              //         sample_pos[2], quality_av);
              frontier.viewpoints_.push_back(vp);
              // int gain = findMaxGainYaw(sample_pos, frontier, sample_yaw);
            }
          }
          else
          {
            if (visib_num > min_visib_num_)
            {
              Viewpoint vp = {sample_pos, avg_yaw, visib_num, 0};
              frontier.viewpoints_.push_back(vp);
            }
          }
          // }
        }
  }

  bool FrontierFinder::isNearObstacle(const Eigen::Vector3d &pos)
  {
    const int vox_num = floor(min_candidate_clearance_ / resolution_);
    for (int x = -vox_num; x <= vox_num; ++x)
      for (int y = -vox_num; y <= vox_num; ++y)
        for (int z = -1; z <= 1; ++z)
        {
          Eigen::Vector3d vox;
          vox << pos[0] + x * resolution_, pos[1] + y * resolution_,
              pos[2] + z * resolution_;
          if (edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::UNKNOWN ||
              edt_env_->sdf_map_->getOccupancy(vox) == SDFMap::OCCUPIED)
            return true;
        }
    return false;
  }

  int FrontierFinder::countVisibleCells(const Eigen::Vector3d &pos,
                                        const double &yaw,
                                        const vector<Eigen::Vector3d> &cluster,
                                        const int ftr_type,
                                        double &quality_av,
                                        bool is_target)
  {
    Eigen::Matrix3d R_wb;
    // std::vector<Eigen::Vector3d> visib_cells;
    R_wb << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;
    Eigen::Quaterniond q(R_wb);
    percep_utils_lidar_->setPose(pos, q);
    int visib_num = 0;
    Eigen::Vector3i idx;
    quality_av = 0;
    for (auto cell : cluster)
    {

      // Check if frontier cell is inside FOV
      if (!percep_utils_lidar_->insideFOV(cell))
        continue;

      // Check if frontier cell is visible (not occulded by obstacles)
      raycaster_->input(cell, pos);
      if (ftr_type == QUALITYFTR)
      {
        // For frontiers with poor quality, raycast one cell first, otherwise it will be directly judged as occupy and not satisfy the condition
        raycaster_->nextId(idx);
      }

      bool visib = true;
      // Eigen::Vector3d visible_pos;
      // std::vector<Eigen::Vector3d> visible_path;

      while (raycaster_->nextId(idx))
      {
        if (edt_env_->sdf_map_->getOccupancy(idx) != SDFMap::FREE ||
            !edt_env_->sdf_map_->isInMap(idx))
        {
          visib = false;
          break;
        }
        // edt_env_->sdf_map_->indexToPos(idx, visible_pos);
        // visible_path.push_back(visible_pos);
      }

      if (visib)
      {
        visib_num += 1;
        if (ftr_type == QUALITYFTR)
        {
          Eigen::Vector3i cell_id;
          edt_env_->sdf_map_->posToIndex(cell, cell_id);
          int cell_adr = edt_env_->sdf_map_->toAddress(cell_id);
          Eigen::Vector3d plane_normal = edt_env_->sdf_map_->getPlaneNormal(cell_adr);
          Eigen::Vector3d dir = (cell - pos).normalized();
          double cos_theta = abs(dir.dot(plane_normal.normalized()));
          if (!isnan(cos_theta))
          {
            quality_av += cos_theta;
          }
        }
        // if (is_target)
        // {
        //   drawDebugInfo(visible_path, 1);
        // }
      }
    }

    quality_av /= visib_num;
    // if (is_target)
    // {
    //   drawDebugInfo(visib_cells, 2);
    // }
    return visib_num;
  }

  void FrontierFinder::wrapYaw(double &yaw)
  {
    while (yaw < -M_PI)
      yaw += 2 * M_PI;
    while (yaw > M_PI)
      yaw -= 2 * M_PI;
  }

  void FrontierFinder::updateFrontierCostMatrix()
  {
    // for (auto ftr : frontiers_)

    if (!removed_ids_.empty())
    {
      // for (int i = 0; i < removed_ids_.size(); ++i)
      // {
      // }
      // Delete path and cost for removed clusters
      for (auto it = frontiers_.begin(); it != first_new_ftr_; ++it)
      {
        auto cost_iter = it->costs_.begin();
        auto path_iter = it->paths_.begin();
        int iter_idx = 0;
        for (int i = 0; i < removed_ids_.size(); ++i)
        {
          // Step iterator to the item to be removed
          while (iter_idx < removed_ids_[i])
          {
            ++cost_iter;
            ++path_iter;
            ++iter_idx;
          }
          cost_iter = it->costs_.erase(cost_iter);
          path_iter = it->paths_.erase(path_iter);
        }
      }
      removed_ids_.clear();
    }

    auto updateCost = [this](const list<Frontier>::iterator &it1, const list<Frontier>::iterator &it2)
    {
      // Search path from old cluster's top viewpoint to new cluster'
      Viewpoint &vui = it1->viewpoints_.front();
      Viewpoint &vuj = it2->viewpoints_.front();
      vector<Vector3d> path_ij;
      double cost_ij = this->computeCost(
          vui.pos_, vuj.pos_, vui.yaw_, vuj.yaw_, Vector3d(0, 0, 0), 0, path_ij);
      // Insert item for both old and new clusters
      it1->costs_.push_back(cost_ij);
      it1->paths_.push_back(path_ij);
      reverse(path_ij.begin(), path_ij.end());
      it2->costs_.push_back(cost_ij);
      it2->paths_.push_back(path_ij);
    };

    // Compute path and cost between old and new clusters
    for (auto it1 = frontiers_.begin(); it1 != first_new_ftr_; ++it1)
      for (auto it2 = first_new_ftr_; it2 != frontiers_.end(); ++it2)
        updateCost(it1, it2);

    // Compute path and cost between new clusters
    for (auto it1 = first_new_ftr_; it1 != frontiers_.end(); ++it1)
      for (auto it2 = it1; it2 != frontiers_.end(); ++it2)
      {
        if (it1 == it2)
        {
          it1->costs_.push_back(0);
          it1->paths_.push_back({});
        }
        else
          updateCost(it1, it2);
      }
    // for (auto ftr : frontiers_)
  }

  void FrontierFinder::getFullCostMatrix(
      const Vector3d &cur_pos, const Vector3d &cur_vel, const Vector3d cur_yaw,
      Eigen::MatrixXd &mat)
  {
    if (false)
    {
      // Use symmetric TSP formulation
      int dim = frontiers_.size() + 2;
      mat.resize(dim, dim); // current pose (0), sites, and virtual depot finally

      int i = 1, j = 1;
      for (auto ftr : frontiers_)
      {
        for (auto cs : ftr.costs_)
          mat(i, j++) = cs;
        ++i;
        j = 1;
      }

      // Costs from current pose to sites
      for (auto ftr : frontiers_)
      {
        Viewpoint vj = ftr.viewpoints_.front();
        vector<Vector3d> path;
        mat(0, j) = mat(j, 0) =
            computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
        ++j;
      }
      // Costs from depot to sites, the same large vaule
      for (j = 1; j < dim - 1; ++j)
      {
        mat(dim - 1, j) = mat(j, dim - 1) = 100;
      }
      // Zero cost to depot to ensure connection
      mat(0, dim - 1) = mat(dim - 1, 0) = -10000;
    }
    else
    {
      // Use Asymmetric TSP
      int dimen = frontiers_.size();
      mat.resize(dimen + 1, dimen + 1);
      // Fill block for clusters
      int i = 1, j = 1;
      for (auto ftr : frontiers_)
      {
        for (auto cs : ftr.costs_)
        {
          // << ", ";
          mat(i, j++) = cs;
        }
        ++i;
        j = 1;
      }

      // Fill block from current state to clusters
      mat.leftCols<1>().setZero();
      for (auto ftr : frontiers_)
      {
        // << ", ";
        Viewpoint vj = ftr.viewpoints_.front();
        vector<Vector3d> path; // The strange values in the first row of mat are computed here
        mat(0, j++) =
            computeCost(cur_pos, vj.pos_, cur_yaw[0], vj.yaw_, cur_vel, cur_yaw[1], path);
      }
    }
  }

  void FrontierFinder::getPathForTour(
      const Vector3d &pos, const vector<int> &frontier_ids, vector<Vector3d> &path)
  {
    // Make an frontier_indexer to access the frontier list easier
    vector<list<Frontier>::iterator> frontier_indexer;
    for (auto it = frontiers_.begin(); it != frontiers_.end(); ++it)
      frontier_indexer.push_back(it);

    // Compute the path from current pos to the first frontier
    vector<Vector3d> segment;
    searchPath(pos, frontier_indexer[frontier_ids[0]]->viewpoints_.front().pos_, segment);
    path.insert(path.end(), segment.begin(), segment.end());

    // Get paths of tour passing all clusters
    for (int i = 0; i < frontier_ids.size() - 1; ++i)
    {
      // Move to path to next cluster
      auto path_iter = frontier_indexer[frontier_ids[i]]->paths_.begin();
      int next_idx = frontier_ids[i + 1];
      for (int j = 0; j < next_idx; ++j)
        ++path_iter;
      path.insert(path.end(), path_iter->begin(), path_iter->end());
    }
  }

  double FrontierFinder::computeCost(const Vector3d &p1, const Vector3d &p2, const double &y1, const double &y2,
                                     const Vector3d &v1, const double &yd1, vector<Vector3d> &path)
  {
    // Cost of position change
    double pos_cost;
    double dist = (p1-p2).norm();
    if(dist > 20.0){
      pos_cost = 1.5 * dist / vm_;
    }
    else{
      pos_cost= searchPath(p1, p2, path) / vm_;
    }

    // Consider velocity change
    if (v1.norm() > 1e-3)
    {
      Vector3d dir = (p2 - p1).normalized();
      Vector3d vdir = v1.normalized();
      double diff = acos(vdir.dot(dir));
      pos_cost += w_dir_ * diff;
      // double vc = v1.dot(dir);
      // pos_cost += w_dir_ * pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
      // if (vc < 0)
      //   pos_cost += w_dir_ * 2 * fabs(vc) / am_;
    }

    // Cost of yaw change
    double diff = fabs(y2 - y1);
    diff = min(diff, 2 * M_PI - diff);
    double yaw_cost = diff / yd_;
    return max(pos_cost, yaw_cost);
    // return pos_cost;
  }

  double FrontierFinder::searchPath(const Vector3d &p1, const Vector3d &p2, vector<Vector3d> &path)
  {
    // Try connect two points with straight line
    bool safe = true;
    Eigen::Vector3i idx;
    raycaster_->input(p1, p2);
    while (raycaster_->nextId(idx))
    {
      if (edt_env_->sdf_map_->getInflateOccupancy(idx) == 1 || edt_env_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN ||
          !edt_env_->sdf_map_->isInBox(idx))
      {
        safe = false;
        break;
      }
    }
    if (safe)
    {
      path = {p1, p2}; // The problem might be here: the value returned when unsafe
      return (p1 - p2).norm();
    }
    // Search a path using decreasing resolution
    vector<double> res = {1.0}; // This number should be made larger; res is 0.5, should be even larger than this
    for (int k = 0; k < res.size(); ++k)
    {
      astar_->reset();
      astar_->setResolution(res[k]);
      if (astar_->search(p1, p2) == Astar::REACH_END)
      {
        path = astar_->getPath();
        return astar_->pathLength(path);
      }
    }
    // Use Astar early termination cost as an estimate
    path = {p1, p2};
    return 1000;
  }

  void FrontierFinder::setTargetFrontier(int id)
  {
    target_frt_id_ = id;
    auto it = frontiers_.begin();
    for (int i = 0; i < id; ++i)
      ++it;
    it->is_target_ = true;
  }

  void FrontierFinder::resetTargetFrontier()
  {
    for (auto frontier : frontiers_)
    {
      frontier.is_target_ = false;
    }
  }

  void FrontierFinder::deactiveTargetFrontier(Eigen::Vector3d pos, double yaw)
  {
    for (auto frontier : frontiers_)
    {
      if (frontier.is_target_)
      {
        double dist = (frontier.viewpoints_[0].pos_ - pos).norm();
        double diff = fabs(yaw - frontier.viewpoints_[0].yaw_);
        diff = min(diff, 2 * M_PI - diff);
        if (dist < 0.5 && diff < 0.1)
        {
          ROS_WARN("FrontierFinder:: Deactive Target Frontier");
          deactive_target_frontier_ = true;
        }
      }
    }
  }

  bool FrontierFinder::haveTargetFrontier()
  {
    for (auto ftr : frontiers_)
    {
      if (ftr.is_target_)
      {
        return true;
      }
    }
    return false;
  }
  // cp change: compute the costmatrix of cp
  void FrontierFinder::getSwarmCostMatrix(const vector<Vector3d> &positions, const vector<Vector3d> &velocities, const vector<double> yaws, Eigen::MatrixXd &mat)
  {

    const int drone_num = positions.size();
    const int ftr_num = frontiers_.size();
    const int dimen = 1 + drone_num + ftr_num;


    mat = Eigen::MatrixXd::Zero(dimen, dimen);

    // Virtual depot to drones
    for (int i = 0; i < drone_num; ++i)
    {
      mat(0, 1 + i) = -1000;
      mat(1 + i, 0) = 1000;
    }
    // Virtual depot to frontiers
    for (int i = 0; i < ftr_num; ++i)
    {
      mat(0, 1 + drone_num + i) = 1000;
      mat(1 + drone_num + i, 0) = 0;
    }

    // Costs between drones
    for (int i = 0; i < drone_num; ++i)
    {
      for (int j = 0; j < drone_num; ++j)
      {
        mat(1 + i, 1 + j) = 10000;
      }
    }


    // Costs from drones to frontiers
    for (int i = 0; i < drone_num; ++i)
    {
      int j = 0;
      for (auto ftr : frontiers_)
      {
        Viewpoint vj = ftr.viewpoints_.front();
        vector<Vector3d> path;
        mat(1 + i, 1 + drone_num + j) =
            computeCost(positions[i], vj.pos_, yaws[0], vj.yaw_, velocities[i], 0.0, path);
        mat(1 + drone_num + j, 1 + i) = 0;
        ++j;
      }
    }

    // Costs between frontiers
    int i = 0, j = 0;
    for (auto ftr : frontiers_)
    {
      for (auto cs : ftr.costs_)
      {
        mat(1 + drone_num + i, 1 + drone_num + j) = cs;
        ++j;
      }
      ++i;
      j = 0;
    }


    // Diag
    for (int i = 0; i < dimen; ++i)
    {
      mat(i, i) = 1000;
    }

  }
  // cp change: compute the costmatrix of cp, adding a cost term that considers hgrid
  void FrontierFinder::getSwarmCostMatrix(const vector<Vector3d> &positions,
                                          const vector<Vector3d> &velocities, const vector<double> &yaws, const vector<int> &ftr_ids,
                                          const vector<Eigen::Vector3d> &grid_pos, Eigen::MatrixXd &mat)
  {

    Eigen::MatrixXd full_mat;
    getSwarmCostMatrix(positions, velocities, yaws, full_mat);


    // Get part of the full matrix according to selected frontier

    const int drone_num = positions.size();
    const int ftr_num = ftr_ids.size();
    int dimen = 1 + drone_num + ftr_num;
    if (!grid_pos.empty())
      dimen += 1;


    mat = Eigen::MatrixXd::Zero(dimen, dimen);

    // Virtual depot to drones
    for (int i = 0; i < drone_num; ++i)
    {
      mat(0, 1 + i) = -1000;
      mat(1 + i, 0) = 1000;
    }


    // Virtual depot to frontiers
    for (int i = 0; i < ftr_num; ++i)
    {
      mat(0, 1 + drone_num + i) = 1000;
      mat(1 + drone_num + i, 0) = 0;
    }


    // Costs between drones
    for (int i = 0; i < drone_num; ++i)
    {
      for (int j = 0; j < drone_num; ++j)
      {
        mat(1 + i, 1 + j) = 10000;
      }
    }


    // Costs from drones to frontiers
    for (int i = 0; i < drone_num; ++i)
    {
      for (int j = 0; j < ftr_num; ++j)
      {
        mat(1 + i, 1 + drone_num + j) = full_mat(1 + i, 1 + drone_num + ftr_ids[j]);
        mat(1 + drone_num + j, 1 + i) = 0;
      }
    }

    // Costs between frontiers
    for (int i = 0; i < ftr_num; ++i)
    {
      for (int j = 0; j < ftr_num; ++j)
      {
        mat(1 + drone_num + i, 1 + drone_num + j) =
            full_mat(1 + drone_num + ftr_ids[i], 1 + drone_num + ftr_ids[j]);
      }
    }

    // Diag
    for (int i = 0; i < dimen; ++i)
    {
      mat(i, i) = 1000;
    }

    // Consider next grid in global tour
    if (!grid_pos.empty())
    {
      // Depot, 1000, -1000
      mat(0, 1 + drone_num + ftr_num) = 1000;
      mat(1 + drone_num + ftr_num, 0) = -1000;

      // Drone
      for (int i = 0; i < drone_num; ++i)
      {
        mat(1 + i, 1 + drone_num + ftr_num) = 1000;
        mat(1 + drone_num + ftr_num, 1 + i) = 1000;
      }

      // Frontier
      vector<Eigen::Vector3d> points, tmps;
      vector<double> yaws;
      getTopViewpointsInfo(positions[0], points, yaws, tmps);
      Eigen::Vector3d next_grid = grid_pos[0];

      for (int i = 0; i < ftr_num; ++i)
      {
        double cost = computeCost(
            next_grid, points[ftr_ids[i]], 0, 0, Eigen::Vector3d(0, 0, 0), 0, tmps);
        mat(1 + drone_num + i, 1 + drone_num + ftr_num) = cost;
        mat(1 + drone_num + ftr_num, 1 + drone_num + i) = cost;
      }
    }
  }
}
