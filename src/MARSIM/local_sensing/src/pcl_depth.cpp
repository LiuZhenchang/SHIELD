#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/range_image/range_image.h> //header file for range images
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h> //header file for range image visualization
#include <pcl/visualization/pcl_visualizer.h>         //header file for PCL visualization
#include <pcl/console/parse.h>

typedef pcl::PointXYZ PointType;
// parameters
float angular_resolution_x = 0.2f, // angular_resolution is the angular resolution of the simulated depth sensor, i.e. the angle that one pixel in the range image corresponds to
    angular_resolution_y = angular_resolution_x;
pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME; // coordinate system that the range image follows
bool live_update = false;
// command help message
void printUsage(const char *progName)
{
  std::cout << "\n\nUsage: " << progName << " [options] <scene.pcd>\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-rx <float>  angular resolution in degrees (default " << angular_resolution_x << ")\n"
            << "-ry <float>  angular resolution in degrees (default " << angular_resolution_y << ")\n"
            << "-c <int>     coordinate frame (default " << (int)coordinate_frame << ")\n"
            << "-l           live update - update the range image according to the selected view in the 3D viewer.\n"
            << "-h           this help\n"
            << "\n\n";
}

void setViewerPose(pcl::visualization::PCLVisualizer &viewer, const Eigen::Affine3f &viewer_pose)
{
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
                           look_at_vector[0], look_at_vector[1], look_at_vector[2],
                           up_vector[0], up_vector[1], up_vector[2]);
}

// main function
int main(int argc, char **argv)
{
  // parse input commands
  if (pcl::console::find_argument(argc, argv, "-h") >= 0)
  {
    printUsage(argv[0]);
    return 0;
  }
  if (pcl::console::find_argument(argc, argv, "-l") >= 0)
  {
    live_update = true;
    std::cout << "Live update is on.\n";
  }
  if (pcl::console::parse(argc, argv, "-rx", angular_resolution_x) >= 0)
    std::cout << "Setting angular resolution in x-direction to " << angular_resolution_x << "deg.\n";
  if (pcl::console::parse(argc, argv, "-ry", angular_resolution_y) >= 0)
    std::cout << "Setting angular resolution in y-direction to " << angular_resolution_y << "deg.\n";
  int tmp_coordinate_frame;
  if (pcl::console::parse(argc, argv, "-c", tmp_coordinate_frame) >= 0)
  {
    coordinate_frame = pcl::RangeImage::CoordinateFrame(tmp_coordinate_frame);
    std::cout << "Using coordinate frame " << (int)coordinate_frame << ".\n";
  }
  angular_resolution_x = pcl::deg2rad(angular_resolution_x);
  angular_resolution_y = pcl::deg2rad(angular_resolution_y);

  // read point cloud PCD file; if no PCD file is provided, generate a point cloud
  pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType> &point_cloud = *point_cloud_ptr;
  Eigen::Affine3f scene_sensor_pose(Eigen::Affine3f::Identity()); // declare the sensor pose as a 4x4 affine transformation
  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument(argc, argv, "pcd");
  if (!pcd_filename_indices.empty())
  {
    std::string filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile(filename, point_cloud) == -1)
    {
      std::cout << "Was not able to open file \"" << filename << "\".\n";
      printUsage(argv[0]);
      return 0;
    }
    // assign values to the sensor pose, i.e. obtain the translation and rotation vectors of the point cloud's sensor
    scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
                                                             point_cloud.sensor_origin_[1],
                                                             point_cloud.sensor_origin_[2])) *
                        Eigen::Affine3f(point_cloud.sensor_orientation_);
  }
  else
  { // if no point cloud is provided, we generate one ourselves
    std::cout << "\nNo *.pcd file given => Genarating example point cloud.\n\n";
    for (float x = -0.5f; x <= 0.5f; x += 0.01f)
    {
      for (float y = -0.5f; y <= 0.5f; y += 0.01f)
      {
        PointType point;
        point.x = x;
        point.y = y;
        point.z = 2.0f - y;
        point_cloud.points.push_back(point);
      }
    }
    point_cloud.width = (int)point_cloud.points.size();
    point_cloud.height = 1;
  }

  // -----obtain the range image from the created point cloud--//
  // set basic parameters
  float noise_level = 0.0;
  float min_range = 0.0f;
  int border_size = 1;
  boost::shared_ptr<pcl::RangeImage> range_image_ptr(new pcl::RangeImage);
  pcl::RangeImage &range_image = *range_image_ptr;
  /*
   Explanation of the parameters of range_image.createFromPointCloud() (all angles involved are in radians):
     point_cloud is the point cloud needed to create the range image
    angular_resolution_x is the angular resolution of the depth sensor in the X direction
    angular_resolution_y is the angular resolution of the depth sensor in the Y direction
     pcl::deg2rad (360.0f) is the maximum horizontal sampling angle of the depth sensor
     pcl::deg2rad (180.0f) is the maximum vertical sampling angle
     scene_sensor_pose is the pose of the simulated sensor, an affine transformation matrix, defaulting to a 4x4 identity transformation
     coordinate_frame defines which coordinate system convention to follow; defaults to CAMERA_FRAME
     noise_level is the influence level of neighboring points on the query point's distance value when obtaining the range image depth
     min_range sets the minimum acquisition distance; positions closer than this are the sensor's blind zone
     border_size sets the width of the border of the obtained range image; defaults to 0
  */
  range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y, pcl::deg2rad(77.0f), pcl::deg2rad(77.0f), scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

  // visualize the point cloud
  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.setBackgroundColor(1, 1, 1);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 0, 0, 0);
  viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
  // viewer.addCoordinateSystem (1.0f, "global");
  // PointCloudColorHandlerCustom<PointType> point_cloud_color_handler (point_cloud_ptr, 150, 150, 150);
  // viewer.addPointCloud (point_cloud_ptr, point_cloud_color_handler, "original point cloud");
  viewer.initCameraParameters();
  // range_image.getTransformationToWorldSystem() obtains the transformation matrix from the range image coordinate system (which should be the sensor coordinate system) to the world coordinate system
  setViewerPose(viewer, range_image.getTransformationToWorldSystem()); // set the viewpoint position

  // visualize the range image
  pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
  range_image_widget.showRangeImage(range_image);

  while (!viewer.wasStopped())
  {
    range_image_widget.spinOnce();
    viewer.spinOnce();
    pcl_sleep(0.01);

    if (live_update)
    {
      // if the -l parameter is selected, it means the range image should be created according to the viewpoint chosen by the user.
      // live update - update the range image according to the selected view in the 3D viewer.
      scene_sensor_pose = viewer.getViewerPose();
      range_image.createFromPointCloud(point_cloud, angular_resolution_x, angular_resolution_y,
                                       pcl::deg2rad(77.0f), pcl::deg2rad(77.0f),
                                       scene_sensor_pose, pcl::RangeImage::LASER_FRAME, noise_level, min_range, border_size);
      range_image_widget.showRangeImage(range_image);
    }
  }
}
