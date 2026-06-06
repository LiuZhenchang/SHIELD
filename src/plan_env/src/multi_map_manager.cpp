#include <plan_env/sdf_map.h>
#include <plan_env/multi_map_manager.h>

namespace shield
{
  MultiMapManager::MultiMapManager()
  {
  }

  MultiMapManager::~MultiMapManager()
  {
  }
  void MultiMapManager::setMap(SDFMap *map)
  {
    this->map_ = map;
  }
  void MultiMapManager::init()
  {
    node_.param("exploration/drone_id", drone_id_, 1);
    node_.param("exploration/vis_drone_id", vis_drone_id_, -1);
    node_.param("exploration/drone_num", map_num_, 2);
    node_.param("multi_map_manager/chunk_size", chunk_size_, 200);

    // chunk_sub_ = node_.subscribe("/multi_map_manager/chunk_data_recv", 5000, &MultiMapManager::chunkCallback, this, ros::TransportHints().tcpNoDelay());

    multi_map_chunks_.resize(map_num_);
    for (auto &data : multi_map_chunks_)
    {
      data.idx_list_ = {};
    }
    chunk_boxes_.resize(map_num_);
    for (auto &box : chunk_boxes_)
    {
      box.valid_ = false;
    }
    buffer_map_.resize(map_num_);
    last_chunk_stamp_time_.resize(map_num_);
    for (auto time : last_chunk_stamp_time_)
      time = 0.0;
  }

  void MultiMapManager::updateMapChunk(const vector<uint32_t> &adrs)
  {
    adr_buffer_.insert(adr_buffer_.end(), adrs.begin(), adrs.end());

    if (adr_buffer_.size() >= chunk_size_)
    {
      // Insert chunk from too long buffer
      int i = 0;
      for (; i + chunk_size_ < adr_buffer_.size(); i += chunk_size_)
      {
        MapChunk chunk;
        chunk.voxel_adrs_.insert(
            chunk.voxel_adrs_.end(), adr_buffer_.begin() + i, adr_buffer_.begin() + i + chunk_size_);
        chunk.idx_ = multi_map_chunks_[drone_id_ - 1].chunks_.size() + 1;
        // if (drone_id_ == 1) std::cout << "Drone 1 insert chunk " << chunk.idx_ << std::endl;
        chunk.need_query_ = true;
        chunk.empty_ = false;
        multi_map_chunks_[drone_id_ - 1].chunks_.push_back(chunk);
      }
      if (multi_map_chunks_[drone_id_ - 1].idx_list_.empty())
      {
        multi_map_chunks_[drone_id_ - 1].idx_list_ = {1, 1};
      }
      multi_map_chunks_[drone_id_ - 1].idx_list_.back() =
          multi_map_chunks_[drone_id_ - 1].chunks_.back().idx_;

      vector<uint32_t> tmp;
      tmp.insert(tmp.end(), adr_buffer_.begin() + i, adr_buffer_.end());
      adr_buffer_ = tmp;
    }
  }

  void MultiMapManager::adrToIndex(const uint32_t &adr, Eigen::Vector3i &idx)
  {
    // x * mp_->map_voxel_num_(1) * mp_->map_voxel_num_(2) + y * mp_->map_voxel_num_(2) + z
    uint32_t tmp_adr = adr;
    const int a = map_->mp_->map_voxel_num_[1] * map_->mp_->map_voxel_num_[2];
    const int b = map_->mp_->map_voxel_num_[2];

    idx[0] = tmp_adr / a;
    tmp_adr = tmp_adr % a;
    idx[1] = tmp_adr / b;
    idx[2] = tmp_adr % b;
  }

  void MultiMapManager::insertChunkToMap(const MapChunk &chunk, const int &drone_id)
  {

    // // Transform from other drone's local frame to this drone's

    for (int i = 0; i < chunk.voxel_adrs_.size(); ++i)
    {
      // Insert occ info

      auto &adr = chunk.voxel_adrs_[i];

      Eigen::Vector3i idx;
      adrToIndex(adr, idx);

      Eigen::Vector3d pos;
      map_->indexToPos(idx, pos);

      // pos = rot * pos + trans;
      if (!map_->isInMap(pos))
        continue;

      map_->posToIndex(pos, idx);
      auto adr_tf = map_->toAddress(idx);

      map_->md_->occupancy_buffer_[adr_tf] =
          chunk.voxel_occ_[i] == 1 ? map_->mp_->clamp_max_log_ : map_->mp_->clamp_min_log_;

      // Update the chunk box

      if (chunk_boxes_[drone_id - 1].valid_)
      {
        for (int k = 0; k < 3; ++k)
        {
          chunk_boxes_[drone_id - 1].min_[k] = min(chunk_boxes_[drone_id - 1].min_[k], pos[k]);
          chunk_boxes_[drone_id - 1].max_[k] = max(chunk_boxes_[drone_id - 1].max_[k], pos[k]);
        }
      }
      else
      {
        chunk_boxes_[drone_id - 1].min_ = chunk_boxes_[drone_id - 1].max_ = pos;
        chunk_boxes_[drone_id - 1].valid_ = true;
      }

      // Update the all box
      for (int k = 0; k < 3; ++k)
      {
        map_->md_->all_min_[k] = min(map_->md_->all_min_[k], pos[k]);
        map_->md_->all_max_[k] = max(map_->md_->all_max_[k], pos[k]);
      }
      // Inflate for the occupied
      if (chunk.voxel_occ_[i] == 1)
      {
        static const int inf_step = ceil(map_->mp_->obstacles_inflation_ / map_->mp_->resolution_);
        for (int inf_x = -inf_step; inf_x <= inf_step; ++inf_x)
          for (int inf_y = -inf_step; inf_y <= inf_step; ++inf_y)
            for (int inf_z = -inf_step; inf_z <= inf_step; ++inf_z)
            {
              Eigen::Vector3i inf_pt(idx[0] + inf_x, idx[1] + inf_y, idx[2] + inf_z);
              if (!map_->isInMap(inf_pt))
                continue;
              int inf_adr = map_->toAddress(inf_pt);
              map_->md_->occupancy_buffer_inflate_[inf_adr] = 1;
            }
      }
    }
  }

  void MultiMapManager::getChunkBoxes(
      vector<Eigen::Vector3d> &mins, vector<Eigen::Vector3d> &maxs, bool reset)
  {
    for (auto &box : chunk_boxes_)
    {
      if (box.valid_)
      {
        mins.push_back(box.min_);
        maxs.push_back(box.max_);
        if (reset)
          box.valid_ = false;
      }
    }
  }

  /******************LZC added program******************/

  void MultiMapManager::testprint()
  {
    printf("Hello world!\n");
  }
  /*****************************************************/

}