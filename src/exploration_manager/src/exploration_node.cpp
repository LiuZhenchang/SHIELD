#include <ros/ros.h>
#include <exploration_manager/shield_exploration_fsm.h>

using namespace shield;

int main(int argc, char** argv) {
  ros::init(argc, argv, "exploration_node");
  // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,ros::console::levels::Debug);

  ros::NodeHandle nh("~");

  FastExplorationFSM expl_fsm;
  expl_fsm.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
