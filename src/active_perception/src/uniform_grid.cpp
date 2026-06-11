#include <active_perception/uniform_grid.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <plan_env/multi_map_manager.h>

namespace shield
{

  UniformGrid::UniformGrid(
      const shared_ptr<EDTEnvironment> &edt, ros::NodeHandle &nh, const int &level)
  {

    this->edt_ = edt;

    // Read min, max, resolution here
    nh.param("sdf_map/box_min_x", min_[0], 0.0);
    nh.param("sdf_map/box_min_y", min_[1], 0.0);
    nh.param("sdf_map/box_min_z", min_[2], 0.0);
    nh.param("sdf_map/box_max_x", max_[0], 0.0);
    nh.param("sdf_map/box_max_y", max_[1], 0.0);
    nh.param("sdf_map/box_max_z", max_[2], 0.0);
    nh.param("sdf_map/scale_x", scale_[0], 1);
    nh.param("sdf_map/scale_y", scale_[1], 1);
    nh.param("sdf_map/scale_z", scale_[2], 1);
    nh.param("partitioning/min_frontier", min_frontier_, 100);
    nh.param("partitioning/min_unknown_percent", min_unknown_percent_, 0.9);
    nh.param("partitioning/min_free_percent", min_free_percent_, 1.0);
    nh.param("partitioning/consistent_cost", consistent_cost_, 3.5);
    nh.param("partitioning/w_unknown", w_unknown_, 3.5);

    double grid_size;
    double grid_height;
    nh.param("partitioning/grid_size", grid_size, 5.0);
    nh.param("partitioning/grid_height", grid_height, -1.0);



    Eigen::Vector3d size = max_ - min_;

    // resolution_ = size / 3;
    for (int i = 0; i < 3; ++i)
    {
      if(i==2 && grid_height > 0){
        grid_size = grid_height;
      }
      int num = ceil(size[i] / grid_size);
      resolution_[i] = size[i] / double(num);
      for (int j = 1; j < level; ++j)
        resolution_[i] *= 0.5;
    }
    initialized_ = false;
    level_ = level;
  }

  UniformGrid::~UniformGrid()
  {
  }

  void UniformGrid::initGridData()
  {
    Eigen::Vector3d size = max_ - min_;
    for (int i = 0; i < 3; ++i)
      grid_num_(i) = ceil(size(i) / resolution_[i]);
    grid_data_.resize(grid_num_[0] * grid_num_[1] * grid_num_[2]);

    std::cout << "data size: " << grid_data_.size() << std::endl;
    std::cout << "grid num: " << grid_num_.transpose() << std::endl;
    std::cout << "resolution: " << resolution_.transpose() << std::endl;

    // Init each grid info
    for (int x = 0; x < grid_num_[0]; ++x)
    {
      for (int y = 0; y < grid_num_[1]; ++y)
      {
        for (int z = 0; z < grid_num_[2]; ++z)
        {
          Eigen::Vector3i id(x, y, z);
          auto &grid = grid_data_[toAddress(id)];

          Eigen::Vector3d pos;
          indexToPos(id, 0.5, pos);

          grid.center_ = pos;
          grid.center_unknown_ = pos;
          grid.center_free_ = pos;
          grid.free_num_ = 0;
          grid.unknown_num_ = resolution_[0] * resolution_[1] * resolution_[2] / pow(edt_->sdf_map_->getResolution(), 3);
          grid.voxel_num_ = grid.unknown_num_; // LZC added

          grid.is_prev_relevant_ = true;
          grid.is_cur_relevant_ = true;
          grid.need_divide_ = false;
          if (level_ == 1)
            grid.type_ = ACTIVE;
          else
            grid.type_ = DORMANT;
        }
      }
    }
  }

  void UniformGrid::updateBaseCoor()
  {
    for (int i = 0; i < grid_data_.size(); ++i)
    {
      auto &grid = grid_data_[i];
      // if (!grid.active_) continue;

      Eigen::Vector3i id;
      adrToIndex(i, id);

      // Compute vertices and box of grid in current drone's frame
      Eigen::Vector3d left_bottom, right_top, left_top, right_bottom;
      indexToPos(id, 0.0, left_bottom);
      indexToPos(id, 1.0, right_top);
      left_top[0] = left_bottom[0];
      left_top[1] = right_top[1];
      left_top[2] = left_bottom[2];
      right_bottom[0] = right_top[0];
      right_bottom[1] = left_bottom[1];
      right_bottom[2] = left_bottom[2];
      right_top[2] = left_bottom[2];

      vector<Eigen::Vector3d> vertices = {left_bottom, right_bottom, right_top, left_top};

      Eigen::Vector3d vmin, vmax;
      vmin = vmax = vertices[0];
      for (int j = 1; j < vertices.size(); ++j)
      {
        for (int k = 0; k < 2; ++k)
        {
          vmin[k] = min(vmin[k], vertices[j][k]);
          vmax[k] = max(vmax[k], vertices[j][k]);
        }
      }
      grid.vertices_ = vertices;
      grid.vmin_ = vmin;
      grid.vmax_ = vmax;

      // Compute normals of four separating lines
      grid.normals_.clear();
      for (int j = 0; j < 4; ++j)
      {
        Eigen::Vector3d dir = (vertices[(j + 1) % 4] - vertices[j]).normalized();
        grid.normals_.push_back(dir);
      }
      // for (auto v : grid.vertices_)
      // for (auto n : grid.normals_)
      //           << std::endl;
    }
  }
  int UniformGrid::getGridtype(const int &id)
  {
    return grid_data_[id].type_;
  }
  
  //Note, this will modify grid_ids
  void UniformGrid::updateGridData(vector<int> &grid_ids)
  {
    for (auto &grid : grid_data_)
    {
      grid.is_updated_ = false;
    }

    bool reset = (level_ == 2);
    Vector3d update_min, update_max;
    edt_->sdf_map_->getUpdatedBox(update_min, update_max, reset);

    // Rediscovered grid
    vector<int> rediscovered_ids;

    auto have_overlap = [](
                            const Vector3d &min1, const Vector3d &max1, const Vector3d &min2, const Vector3d &max2)
    {
      for (int m = 0; m < 2; ++m)
      {
        double bmin = max(min1[m], min2[m]);
        double bmax = min(max1[m], max2[m]);
        if (bmin > bmax + 1e-3)
          return false;
      }
      return true;
    };

    // For each grid, check overlap with updated box and update it if necessary
    for (int i = 0; i < grid_data_.size(); ++i)
    {
      auto &grid = grid_data_[i];
      if (grid.type_==DEACTIVE)
        continue;

      // Check overlap with updated boxes
      bool overlap_with_fov = have_overlap(grid.vmin_, grid.vmax_, update_min, update_max);
      if (!overlap_with_fov)
        continue;

      // Update the grid
      Eigen::Vector3i idx;
      adrToIndex(i, idx);
      updateGridInfo(idx);
    }

    // Update the list of relevant grid
    relevant_id_.clear();
    relevant_map_.clear();
    for (int i = 0; i < grid_data_.size(); ++i)
    {
      if (isRelevant(grid_data_[i]))
      {
        relevant_id_.push_back(i);
        relevant_map_[i] = 1;
      }
    }

    // Update the dominance grid of ego drone
    if (!initialized_)
    {
      grid_ids = relevant_id_;
      ROS_WARN("Init grid allocation.");
      initialized_ = true;
    }
  }

  void UniformGrid::updateGridInfo(const Eigen::Vector3i &id)
  {
    int adr = toAddress(id);
    auto &grid = grid_data_[adr];
    if (grid.is_updated_)
    { // Ensure only one update to avoid repeated computation
      return;
    }
    grid.is_updated_ = true;

    grid.is_prev_relevant_ = grid.is_cur_relevant_;

    Eigen::Vector3d gmin, gmax;
    // indexToPos(id, 0.0, gmin);
    indexToPos(id, 1.0, gmax); // Only the first 2 values of vmax is useful, should compute max here

    // Check if a voxel is inside the rotated box
    auto inside_box = [](const Eigen::Vector3d &vox, const GridInfo &grid)
    {
      // Check four separating planes(lines)
      for (int m = 0; m < 4; ++m)
      {
        if ((vox - grid.vertices_[m]).dot(grid.normals_[m]) <= 0.0)
          return false;
      }
      return true;
    };

    // Count known
    const double res = edt_->sdf_map_->getResolution();
    grid.free_num_ = 0;
    grid.unknown_num_ = 0;
    grid.center_free_.setZero();
    grid.center_unknown_.setZero();

    for (double x = grid.vmin_[0]; x <= grid.vmax_[0]; x += res * scale_[0])
    {
      for (double y = grid.vmin_[1]; y <= grid.vmax_[1]; y += res * scale_[1])
      {
        for (double z = grid.vmin_[2]; z <= gmax[2]; z += res * scale_[2])
        {

          Eigen::Vector3d pos(x, y, z);
          if (!inside_box(pos, grid))
            continue;

          int state = edt_->sdf_map_->getOccupancy(pos);
          if (state == SDFMap::FREE)
          {
            grid.center_free_ = (grid.center_free_ * grid.free_num_ + pos) / (grid.free_num_ + 1);
            grid.free_num_ += 1;
          }
          else if (state == SDFMap::UNKNOWN)
          {
            grid.center_unknown_ = (grid.center_unknown_ * grid.unknown_num_ + pos) / (grid.unknown_num_ + 1);
            grid.unknown_num_ += 1;
          }
        }
      }
    }

    grid.is_cur_relevant_ = isRelevant(grid);
  }

  int UniformGrid::toAddress(const Eigen::Vector3i &id)
  {
    return id[0] * grid_num_(1) * grid_num_(2) + id[1] * grid_num_(2) + id[2];
  }

  void UniformGrid::adrToIndex(const int &adr, Eigen::Vector3i &idx)
  {
    // id[0] * grid_num_(1) * grid_num_(2) + id[1] * grid_num_(2) + id[2];
    int tmp_adr = adr;
    const int a = grid_num_(1) * grid_num_(2);
    const int b = grid_num_(2);

    idx[0] = tmp_adr / a;
    tmp_adr = tmp_adr % a;
    idx[1] = tmp_adr / b;
    idx[2] = tmp_adr % b;
  }

  void UniformGrid::posToIndex(const Eigen::Vector3d &pos, Eigen::Vector3i &id)
  {
    for (int i = 0; i < 3; ++i)
      id(i) = floor((pos(i) - min_(i)) / resolution_[i]);
  }

  void UniformGrid::indexToPos(const Eigen::Vector3i &id, const double &inc, Eigen::Vector3d &pos)
  {
    // inc: 0 for min, 1 for max, 0.5 for mid point
    for (int i = 0; i < 3; ++i)
      pos(i) = (id(i) + inc) * resolution_[i] + min_(i);
  }

  void UniformGrid::activateGrids(const vector<int> &ids)
  {
    for (auto id : ids)
    {
      grid_data_[id].type_ = ACTIVE;
    }
    extra_ids_ = ids; // To avoid incomplete update
  }
  void UniformGrid::deactivateGrids(const vector<int> &ids)
  {
    for (auto id : ids)
    {
      grid_data_[id].type_ = DEACTIVE;
    }
  }

  void UniformGrid::deactivateGrid(int &id)
  {
    grid_data_[id].type_ = DEACTIVE;
  }

  void UniformGrid::activateGrid(int &id)
  {
    grid_data_[id].type_ = ACTIVE;
  }

  void UniformGrid::dormantGrid(int &id)
  {
    grid_data_[id].type_ = DORMANT;
  }

  bool UniformGrid::insideGrid(const Eigen::Vector3i &id)
  {
    // Check inside min max
    for (int i = 0; i < 3; ++i)
    {
      if (id[i] < 0 || id[i] >= grid_num_[i])
      {
        return false;
      }
    }
    return true;
  }

  void UniformGrid::inputFrontiers(const vector<Eigen::Vector3d> &avgs)
  {
    for (auto &grid : grid_data_)
    {
      grid.contained_frontier_ids_.clear();
      grid.frontier_num_ = 0;
    }
    Eigen::Vector3i id;


    for (int i = 0; i < avgs.size(); ++i)
    {
      Eigen::Vector3d pos = avgs[i];

      posToIndex(pos, id);
      if (!insideGrid(id))
        continue;
      auto &grid = grid_data_[toAddress(id)];
      grid.contained_frontier_ids_[i] = 1;
      grid.frontier_num_ += 1;//count the number of frts in the grid
    }
  }

  void UniformGrid::inputFrontiers(const vector<Frontier> &frts){
    for (auto &grid : grid_data_)
    {
      grid.contained_frontier_ids_.clear();
      grid.frontier_num_ = 0;
      grid.frontier_quality_num_ = 0;
      grid.frontier_unknown_num_ = 0;
    }
    Eigen::Vector3i id;


    for (int i = 0; i < frts.size(); ++i)
    {
      Eigen::Vector3d pos = frts[i].average_;
      posToIndex(pos, id);
      if (!insideGrid(id)){
        continue;
      }
      auto &grid = grid_data_[toAddress(id)];
      grid.contained_frontier_ids_[i] = 1;
      grid.frontier_num_ += 1;//count the number of frts in the grid
      //enum FrontierTYPE { FREEUNKNOWNFTR = 0, QUALITYFTR =1};
      if(frts[i].type_ == 1){
        grid.frontier_quality_num_ += 1;
      }
      else if(frts[i].type_ == 0){
        grid.frontier_unknown_num_ += 1;
      }
    }    
  }


  bool UniformGrid::isRelevant(const GridInfo &grid)
  {
    return grid.unknown_num_ >= grid.voxel_num_ * min_unknown_percent_ || !grid.contained_frontier_ids_.empty();
  }

  void UniformGrid::getGridTour(const vector<int> &ids, vector<Eigen::Vector3d> &tour)
  {
    tour.clear();
    for (int i = 0; i < ids.size(); ++i)
    {
      tour.push_back(grid_data_[ids[i]].center_);
    }
  }

  void UniformGrid::getFrontiersInGrid(const int &grid_id, vector<int> &ftr_ids)
  {
    // Find frontier having more than 1/4 within the first grid
    auto &first_grid = grid_data_[grid_id];
    ftr_ids.clear();
    // for (auto pair : first_grid.frontier_cell_nums_) {
    //   ftr_ids.push_back(pair.first);
    // }
    for (auto pair : first_grid.contained_frontier_ids_)
    {
      ftr_ids.push_back(pair.first);
    }
  }

  void UniformGrid::getGridMarker(vector<Eigen::Vector3d> &pts1, vector<Eigen::Vector3d> &pts2)
  {

    Eigen::Vector3d p1 = min_;
    Eigen::Vector3d p2 = min_ + Eigen::Vector3d(max_[0] - min_[0], 0, 0);
    for (int i = 0; i <= grid_num_[1]; ++i)
    {
      Eigen::Vector3d pt1 = p1 + Eigen::Vector3d(0, resolution_[1] * i, 0);
      Eigen::Vector3d pt2 = p2 + Eigen::Vector3d(0, resolution_[1] * i, 0);
      pts1.push_back(pt1);
      pts2.push_back(pt2);
    }

    p1 = min_;
    p2 = min_ + Eigen::Vector3d(0, max_[1] - min_[1], 0);
    for (int i = 0; i <= grid_num_[0]; ++i)
    {
      Eigen::Vector3d pt1 = p1 + Eigen::Vector3d(resolution_[0] * i, 0, 0);
      Eigen::Vector3d pt2 = p2 + Eigen::Vector3d(resolution_[0] * i, 0, 0);
      pts1.push_back(pt1);
      pts2.push_back(pt2);
    }
    for (auto &p : pts1)
      p[2] = 0.5;
    for (auto &p : pts2)
      p[2] = 0.5;
  }

  /********************* LZC Added **********************/
  void UniformGrid::resetGridPathInfo()
  {
    for (auto &grid : grid_data_)
    {
      grid.path_node_.clear();
      grid.path_length_ = -1;
      grid.uav_distance_ = -1;
      grid.path_status_ = grid.UNPLAN;
    }
  }

  void UniformGrid::getExploredGrids(vector<int> &explored_ids)
  {
    explored_ids.clear();

    for (int i = 0; i < grid_data_.size(); ++i)
    {
      if (!grid_data_[i].is_cur_relevant_ || !isRelevant(grid_data_[i]))
      {
        if (!grid_data_[i].need_divide_)
        {
          explored_ids.push_back(i);
        }
      }
    }
  }

  bool UniformGrid::checkPointRange(const Eigen::Vector3d &pos)
  {
    bool flag = true;
    for (int i = 0; i < 3; i++)
    {
      if (pos(i) < min_(i) || pos(i) > max_(i))
        flag = false;
    }
    return flag;
  }

  int UniformGrid::getGridNum(){
    return grid_data_.size();
  }

  int UniformGrid::getGridStatus(const int &grid_id){
    return grid_data_[grid_id].type_;
  }


} // namespace shield
