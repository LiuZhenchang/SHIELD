#ifndef _EDT_ENVIRONMENT_H_
#define _EDT_ENVIRONMENT_H_

#include <Eigen/Eigen>
#include <iostream>
#include <utility>
#include <ros/ros.h>


using std::cout;
using std::endl;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

namespace shield {
class SDFMap;

class EDTEnvironment {
private:
  /* data */

  double resolution_inv_;
  double distToBox(int idx, const Eigen::Vector3d& pos, const double& time);
  double minDistToAllBox(const Eigen::Vector3d& pos, const double& time);

public:
  EDTEnvironment(/* args */) {
  }
  ~EDTEnvironment() {
  }

  shared_ptr<SDFMap> sdf_map_;

  void init();
  void setMap(shared_ptr<SDFMap>& map);

  void evaluateEDTWithGrad(
      const Eigen::Vector3d& pos, double time, double& dist, Eigen::Vector3d& grad);


  // deprecated
  void interpolateTrilinear(
      double values[2][2][2], const Eigen::Vector3d& diff, double& value, Eigen::Vector3d& grad);

  typedef shared_ptr<EDTEnvironment> Ptr;
};

}  // namespace shield

#endif