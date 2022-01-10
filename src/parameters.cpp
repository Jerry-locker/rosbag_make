#include "parameters.h"

std::string GS_CAM_PATH;
std::string RS_CAM_PATH;
std::string IMU_PATH;

std::string GS_IMAGE_TOPIC;
std::string RS_IMAGE_TOPIC;
std::string IMU_TOPIC;
std::string TOBAG_PATH;

template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
  T ans;
  if (n.getParam(name, ans))
  {
    ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load " << name);
    n.shutdown();
  }
  return ans;
}

void readParameters(ros::NodeHandle &n)
{
  std::string config_file = readParam<std::string>(n, "config_file");
  std::cout << "[config_file] " << config_file << std::endl;
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if(!fsSettings.isOpened())
  {
      std::cerr << "ERROR: Wrong path to settings" << std::endl;
  }

  fsSettings["gs_cam_path"] >> GS_CAM_PATH;
  fsSettings["rs_cam_path"] >> RS_CAM_PATH;
  fsSettings["imu_path"] >> IMU_PATH;

  fsSettings["gs_image_topic"] >> GS_IMAGE_TOPIC;
  fsSettings["rs_image_topic"] >> RS_IMAGE_TOPIC;
  fsSettings["imu_topic"] >> IMU_TOPIC;
  fsSettings["tobag_path"] >> TOBAG_PATH;
}
