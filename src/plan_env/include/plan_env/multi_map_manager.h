#ifndef _MULTI_MAP_MANAGER_H
#define _MULTI_MAP_MANAGER_H

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <memory>
#include <random>
#include <vector>
#include <unordered_map>

using std::shared_ptr;
using std::unordered_map;
using std::vector;

namespace shield
{
  class SDFMap;
  class MapROS;

  // A map chunk, the elementary exchange unit between robots
  struct MapChunk
  {
    // double stamp_;
    uint32_t idx_; // Start from 1
    vector<uint32_t> voxel_adrs_;
    vector<uint8_t> voxel_occ_;

    bool need_query_;
    bool empty_;
  };

  struct ChunksData
  {
    vector<MapChunk> chunks_;
    // uint32_t latest_idx_;
    vector<int> idx_list_;
    // double latest_stamp_;
  };

  struct ChunksBox
  {
    Eigen::Vector3d min_;
    Eigen::Vector3d max_;
    bool valid_;
  };

  class MultiMapManager
  {
  public:
    MultiMapManager();
    ~MultiMapManager();
    void setMap(SDFMap *map);
    void init();
    void updateMapChunk(const vector<uint32_t>& adrs);
    void getChunkBoxes(
        vector<Eigen::Vector3d> &mins, vector<Eigen::Vector3d> &maxs, bool reset = true);
    
    /******************LZC added program******************/
    void testprint();
    /*****************************************************/

  private:
    void insertChunkToMap(const MapChunk &chunk, const int &chunk_drone_id);
    void adrToIndex(const uint32_t &adr, Eigen::Vector3i &idx);

    // data----------------

    int drone_id_;
    int map_num_;
    int vis_drone_id_; // ONLY use for ground node!
    int chunk_size_;

    SDFMap *map_;
    ros::NodeHandle node_;
    ros::Subscriber chunk_sub_;
    ros::Timer chunk_timer_;

    // Main map data
    vector<ChunksData> multi_map_chunks_;
    // Buffer for chunks of this map
    vector<uint32_t> adr_buffer_;
    // Hash map to avoid repeated insertion of chunk msg
    vector<unordered_map<int, char>> buffer_map_;

    vector<double> last_chunk_stamp_time_;

    // Bounding box of map chunks of swarm
    vector<ChunksBox> chunk_boxes_;

    friend SDFMap;
    friend MapROS;
  };
}
#endif