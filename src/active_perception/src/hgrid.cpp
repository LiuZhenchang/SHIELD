#include <active_perception/uniform_grid.h>
#include <active_perception/hgrid.h>
#include <path_searching/astar2.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>

#include <visualization_msgs/Marker.h>

namespace shield
{
    HGrid::HGrid(const shared_ptr<EDTEnvironment> &edt, ros::NodeHandle &nh)
    {

        this->edt_ = edt;
        path_finder_.reset(new Astar);
        path_finder_->init(nh, edt);
        // path_finder_->setResolution(2.0);

        grid_.reset(new UniformGrid(edt, nh, 1));
        grid_->initGridData();
        grid_->updateBaseCoor();

        nh.param("hgrid/unknown_percent_thresh", unknown_percent_thresh_, 0.01);
        nh.param("hgrid/coefficient_vel_cost", coefficient_vel_cost_, 1.0);
        nh.param("hgrid/coefficient_frt_cost", coefficient_frt_cost_, 1.0);


    }

    HGrid::~HGrid()
    {
    }

    void HGrid::setDronePos(const Eigen::Vector3d pos){
        drone_pos_ = pos;
    }

    int HGrid::getDroneGridID() {
        return getGridIDofPos(drone_pos_);
    }

    void HGrid::updateGridData(vector<int> &grid_ids)
    {
        grid_->updateGridData(grid_ids);

        for(auto id =0; id < getHGridNum(); id++){
            //cp更改：在这里deactivate满足条件的hgrid
            //如果未知区域占比小于0.1，就 deactivate
            double unknown_percent = get_unknown_percent(id);
            int frontier_num = getFrontierNum(id);
            int frontier_qua_num = getQualityFrontierNum(id);
            int frontier_unknown_num = getUnknownFrontierNum(id);
            int type = grid_->getGridtype(id);
            switch (type){
                case UniformGrid::ACTIVE:
                    if(unknown_percent < unknown_percent_thresh_){
                        grid_->deactivateGrid(id);
                    }
                    if(frontier_num == 0){
                        grid_->dormantGrid(id);
                    }
                    break;
                case UniformGrid::DORMANT:
                    if(frontier_unknown_num > 0 || frontier_qua_num > 2){
                        grid_->activateGrid(id);
                    }
                    break;
                case UniformGrid::DEACTIVE:
                    break;

                default:
                    grid_->activateGrid(id);
                    break;
            }
            
        }
    }

    void HGrid::getActiveGrids(vector<int> &grid_ids)
    {
        grid_ids.clear();
        const int grid_num = grid_->grid_data_.size();
        for (int i = 0; i < grid_num; ++i)
        {
            if (grid_->grid_data_[i].type_ == UniformGrid::ACTIVE)
            {
                grid_ids.push_back(i);
            }
        }
    }

    void HGrid::deActivateGrid(int &id)
    {
        grid_->deactivateGrid(id);
    }

    void HGrid::dormantGrid(int &id)
    {
        grid_->dormantGrid(id);
    }

    Eigen::Vector3d HGrid::getCenter(const int &id, int type)
    {
        Eigen::Vector3d center;
        switch (type)
        {
        case 0:
            center = grid_->grid_data_[id].center_;
            break;
        case 1:
            center = grid_->grid_data_[id].center_free_;
            break;
        case 2:
            center = grid_->grid_data_[id].center_unknown_;
            break;
        }
        return center;
    }

    void HGrid::getCenterAll(vector<Eigen::Vector3d> &centers, vector<int> &ids, int type)
    {
        centers.clear();
        ids.clear();
        switch (type)
        {
        case 0:
            for (int i = 0; i < grid_->grid_data_.size(); i++)
            {
                centers.push_back(grid_->grid_data_[i].center_);
                ids.push_back(i);
            }
            break;
        case 1:
            for (int i = 0; i < grid_->grid_data_.size(); i++)
            {
                if (grid_->grid_data_[i].free_num_ == 0)
                    continue;
                centers.push_back(grid_->grid_data_[i].center_free_);
                ids.push_back(i);
            }
            break;
        case 2:
            for (int i = 0; i < grid_->grid_data_.size(); i++)
            {
                if (grid_->grid_data_[i].unknown_num_ == 0)
                    continue;
                centers.push_back(grid_->grid_data_[i].center_unknown_);
                ids.push_back(i);
            }
            break;
        }
    }

    Eigen::Vector3d HGrid::getResolution()
    {
        return grid_->resolution_;
    }

    GridInfo &HGrid::getGrid(const int &id)
    {

        return grid_->grid_data_[id];
    }

    int HGrid::getFrontierNum(const int &id)
    {
        return grid_->grid_data_[id].frontier_num_;
    }

    int HGrid::getUnknownFrontierNum(const int &id)
    {
        return grid_->grid_data_[id].frontier_unknown_num_;
    }

    int HGrid::getQualityFrontierNum(const int &id)
    {
        return grid_->grid_data_[id].frontier_quality_num_;
    }

    double HGrid::get_unknown_percent(const int &id)
    {
        int unknown_num = grid_->grid_data_[id].unknown_num_;
        int free_num = grid_->grid_data_[id].free_num_;
        return double(unknown_num) / double(unknown_num + free_num);
    }

    double HGrid::getCostDroneToGrid(
        const Eigen::Vector3d &pos, const int &grid_id)
    {

        int current_id = getGridIDofPos(pos);
        
        auto &grid = getGrid(grid_id);
        double dist1, cost;
        dist1 = (pos - grid.center_free_).norm();
        if(current_id == grid_id){
            cost = 0.0;
        }
            else{
            if (dist1 < 5.0)
            {
                path_finder_->reset();
                if (path_finder_->search(pos, grid.center_free_) == Astar::REACH_END)
                {
                    auto path = path_finder_->getPath();
                    cost = path_finder_->pathLength(path);
                }
                else
                {
                    cost = dist1;
                }
            }
            else
            {
                cost = 1.5 * dist1;
            }
        }


        int frt_num = getFrontierNum(grid_id);
        // if(frt_num == 0){
        //     return cost;
        // }
        // else{
        //     return cost/double(frt_num);
        // }
        return cost - coefficient_frt_cost_ * frt_num * 0.2 * grid_->resolution_.norm();
        
    }

    double HGrid::getCostDroneToGrid(
        const Eigen::Vector3d &pos, const Eigen::Vector3d &Vel, const int &grid_id)
    {
        int current_id = getGridIDofPos(pos);


        auto &grid = getGrid(grid_id);
        double dist1, cost;
        dist1 = (pos - grid.center_free_).norm();

        Eigen::Vector3d dir = (grid.center_free_ - pos);

        double cost_vel = std::clamp(dir.dot(Vel.normalized()), - 0.2 * grid_->resolution_.norm(), 0.2 * grid_->resolution_.norm()) * coefficient_vel_cost_;
        if(current_id == grid_id){
            cost = 0.0;
        }
        else{
            if (dist1 < grid_->resolution_.norm())
            {
                if(edt_->sdf_map_->getOccupancy(grid.center_free_) == SDFMap::FREE){//center是free，才进行路径搜索
                    path_finder_->reset();
                    if (path_finder_->search(pos, grid.center_free_) == Astar::REACH_END)
                    {
                        auto path = path_finder_->getPath();
                        cost = path_finder_->pathLength(path) - cost_vel;
                    }
                    else
                    {
                        cost = dist1  - cost_vel;
                    }
                }
                else{
                    cost = dist1  - cost_vel;
                }
                
            }
            else
            {
                cost = 1.5 * (dist1  - cost_vel);
            }
        }

        
        int frt_num = getUnknownFrontierNum(grid_id);
        return cost - coefficient_frt_cost_ * std::clamp(frt_num * 0.2 * grid_->resolution_.norm(), 0.0, 0.5 * grid_->resolution_.norm());
        // return cost;
        // return cost;
    }

    // double HGrid::getCostGridToGrid(const int &id1, const int &id2)
    // {
    //     auto &grid1 = getGrid(id1);
    //     auto &grid2 = getGrid(id2);
    //     double dist_cost = 0.0;

    //     double dist = (grid1.center_ - grid2.center_).norm();
    //     if(edt_->sdf_map_->getOccupancy(grid1.center_free_) == SDFMap::FREE && edt_->sdf_map_->getOccupancy(grid2.center_free_) == SDFMap::FREE){//两个center都是free，才进行路径搜索

    //         if (dist < grid_->resolution_.norm())
    //         {
    //             // Neighbor grid, search path to compute exact cost
    //             path_finder_->reset();
    //             if (path_finder_->search(grid1.center_free_, grid2.center_free_) == Astar::REACH_END)
    //             {
    //                 auto path = path_finder_->getPath();
    //                 dist_cost = path_finder_->pathLength(path);
    //                 // double res = path_finder_->getResolution();
    //             }
    //             else
    //             {
    //                 dist_cost = (grid1.center_free_ - grid2.center_free_).norm();
    //             }
    //         }
    //         else
    //         {
    //             dist_cost = 1.5 * (grid1.center_free_ - grid2.center_free_).norm();
    //             // double dist_cost = (grid1.center_ - grid2.center_).norm();
    //         }
    //     }
    //     else
    //     {
    //         dist_cost = 1.5 * (grid1.center_free_ - grid2.center_free_).norm();
    //         // double dist_cost = (grid1.center_ - grid2.center_).norm();
    //     }

    //     int frt_num2 = getFrontierNum(id2);

    //     return dist_cost - std::clamp(frt_num2 * 0.1 * grid_->resolution_.norm(), 0.0, 0.5 * grid_->resolution_.norm());
    //     // return dist_cost;
    // }

    //直接使用未知区域中心的距离作为成本
    double HGrid::getCostGridToGrid(const int &id1, const int &id2)
    {
        auto &grid1 = getGrid(id1);
        auto &grid2 = getGrid(id2);
        double dist_cost = (grid1.center_free_ - grid2.center_free_).norm();

        int frt_num2 = getUnknownFrontierNum(id2);

        return dist_cost - coefficient_frt_cost_ * std::clamp(frt_num2 * 0.2 * grid_->resolution_.norm(), 0.0, 0.5 * grid_->resolution_.norm());
        // return dist_cost;
    }
    void HGrid::getCostMatrix(const Eigen::Vector3d &position, const vector<int> &grid_ids, Eigen::MatrixXd &mat)
    {
        // first_ids and second_ids are 1 x 1-4 vectors

        // Fill the cost matrix
        const int grid_num = grid_ids.size();
        const int dimen = 1 + grid_num;
        mat = Eigen::MatrixXd::Zero(dimen, dimen);

        // for (auto ids : first_ids)
        //   for (auto id : ids)

        // for (auto ids : second_ids)
        //   for (auto id : ids)

        // Virtual depot to drones
        for (int i = 0; i < grid_num; ++i)
        {
            // struct timeval t1, t2;
            // gettimeofday(&t1, NULL);

            mat(0, 1 + i) = getCostDroneToGrid(position, grid_ids[i]);

            // gettimeofday(&t2, NULL);
            // long seconds = t2.tv_sec - t1.tv_sec;
            // long usec = t2.tv_usec - t1.tv_usec;
            // double elapsed = seconds + usec / 1e6;
        }
        // Costs between grid
        for (int i = 0; i < grid_num; ++i)
        {
            for (int j = i + 1; j < grid_num; ++j)
            {
                // struct timeval t1, t2;
                // gettimeofday(&t1, NULL);

                double cost = getCostGridToGrid(grid_ids[i], grid_ids[j]);
                mat(1 + i, 1 + j) = cost;
                double cost2 = getCostGridToGrid(grid_ids[j], grid_ids[i]);
                mat(1 + j, 1 + i) = cost2;

                // gettimeofday(&t2, NULL);
                // long seconds = t2.tv_sec - t1.tv_sec;
                // long usec = t2.tv_usec - t1.tv_usec;
                // double elapsed = seconds + usec / 1e6;
            }
        }
    }

    void HGrid::getCostMatrix(const Eigen::Vector3d &position, const Eigen::Vector3d &Vel, const vector<int> &grid_ids, Eigen::MatrixXd &mat)
    {
        // first_ids and second_ids are 1 x 1-4 vectors

        // Fill the cost matrix
        const int grid_num = grid_ids.size();
        const int dimen = 1 + grid_num;
        mat = Eigen::MatrixXd::Zero(dimen, dimen);

        // for (auto ids : first_ids)
        //   for (auto id : ids)

        // for (auto ids : second_ids)
        //   for (auto id : ids)

        // Virtual depot to drones
        for (int i = 0; i < grid_num; ++i)
        {
            struct timeval t1, t2;
            gettimeofday(&t1, NULL);

            mat(0, 1 + i) = getCostDroneToGrid(position, Vel, grid_ids[i]);

            gettimeofday(&t2, NULL);
            long seconds = t2.tv_sec - t1.tv_sec;
            long usec = t2.tv_usec - t1.tv_usec;
            double elapsed = seconds + usec / 1e6;
        }
        // Costs between grid
        for (int i = 0; i < grid_num; ++i)
        {
            for (int j = i + 1; j < grid_num; ++j)
            {
                struct timeval t1, t2;
                gettimeofday(&t1, NULL);

                double cost = getCostGridToGrid(grid_ids[i], grid_ids[j]);
                mat(1 + i, 1 + j) = cost;
                double cost2 = getCostGridToGrid(grid_ids[j], grid_ids[i]);
                mat(1 + j, 1 + i) = cost2;

                gettimeofday(&t2, NULL);
                long seconds = t2.tv_sec - t1.tv_sec;
                long usec = t2.tv_usec - t1.tv_usec;
                double elapsed = seconds + usec / 1e6;
            }
        }
    }

    // 读取grid_id中第一个里面包含的frt编号
    void HGrid::getFrontiersInGrid(const vector<int> &grid_ids, vector<int> &frt_ids)
    {
        frt_ids.clear();
        auto id = grid_ids.front();
        auto &grid = grid_->grid_data_[id];
        for (auto pair : grid.contained_frontier_ids_)
            frt_ids.push_back(pair.first);
    }

    void HGrid::inputFrontiers(const vector<Eigen::Vector3d> &avgs)
    {
        // Input frontier to both levels
        grid_->inputFrontiers(avgs);
    }

    void HGrid::inputFrontiers(const vector<Frontier> &frts)
    {
        // Input frontier to both levels
        grid_->inputFrontiers(frts);
    }

    void HGrid::getGridTour(const vector<int> &ids, const Eigen::Vector3d &pos,
                            vector<Eigen::Vector3d> &tour)
    {
        // Get the centers of the visited grids
        vector<Eigen::Vector3d> centers = {pos};
        for (auto id : ids)
        {
            centers.push_back(grid_->grid_data_[id].center_unknown_);
        }
        tour = centers;
    }

    int HGrid::getHGridNum(void){
        int num = grid_->getGridNum();
        return num;
    }

    int HGrid::getHGridStatus(const int &grid_id){
        int status = grid_->getGridStatus(grid_id);
        return status;
    }

    //根据position返回grid id
    int HGrid::getGridIDofPos(const Eigen::Vector3d &pos){
        Eigen::Vector3i id;
        grid_->posToIndex(pos, id);
        int grid_id = grid_->toAddress(id);
        return grid_id;
    }

}