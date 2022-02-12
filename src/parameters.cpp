#include "parameters.h"

std::string GS_CAM_PATH;
std::string RS_CAM_PATH;
std::string IMU90_PATH;
std::string IMU120_PATH;
std::string IMU150_PATH;
std::string IMU180_PATH;
std::string IMU240_PATH;
std::string IMU300_PATH;
std::string IMU360_PATH;
std::string IMU480_PATH;
std::string IMU600_PATH;

std::string GS_IMAGE_TOPIC;
std::string RS_IMAGE_TOPIC;
std::string IMU90_TOPIC;
std::string IMU120_TOPIC;
std::string IMU150_TOPIC;
std::string IMU180_TOPIC;
std::string IMU240_TOPIC;
std::string IMU300_TOPIC;
std::string IMU360_TOPIC;
std::string IMU480_TOPIC;
std::string IMU600_TOPIC;

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

  fsSettings["imu90_path"] >> IMU90_PATH;
  fsSettings["imu120_path"] >> IMU120_PATH;
  fsSettings["imu150_path"] >> IMU150_PATH;
  fsSettings["imu180_path"] >> IMU180_PATH;
  fsSettings["imu240_path"] >> IMU240_PATH;
  fsSettings["imu300_path"] >> IMU300_PATH;
  fsSettings["imu360_path"] >> IMU360_PATH;
  fsSettings["imu480_path"] >> IMU480_PATH;
  fsSettings["imu600_path"] >> IMU600_PATH;

  fsSettings["gs_image_topic"] >> GS_IMAGE_TOPIC;
  fsSettings["rs_image_topic"] >> RS_IMAGE_TOPIC;
  fsSettings["imu90_topic"] >> IMU90_TOPIC;
  fsSettings["imu120_topic"] >> IMU120_TOPIC;
  fsSettings["imu150_topic"] >> IMU150_TOPIC;
  fsSettings["imu180_topic"] >> IMU180_TOPIC;
  fsSettings["imu240_topic"] >> IMU240_TOPIC;
  fsSettings["imu300_topic"] >> IMU300_TOPIC;
  fsSettings["imu360_topic"] >> IMU360_TOPIC;
  fsSettings["imu480_topic"] >> IMU480_TOPIC;
  fsSettings["imu600_topic"] >> IMU600_TOPIC;

  fsSettings["tobag_path"] >> TOBAG_PATH;
}
