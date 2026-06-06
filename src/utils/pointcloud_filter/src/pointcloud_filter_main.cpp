
#include "pointcloud_filter.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_filter");

    pointcloud_filter filter;
    filter.init();

    ros::spin();
    return 0;
}
