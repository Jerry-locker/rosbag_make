#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

extern std::string GS_CAM_PATH;
extern std::string RS_CAM_PATH;
extern std::string IMU90_PATH;
extern std::string IMU120_PATH;
extern std::string IMU150_PATH;
extern std::string IMU180_PATH;
extern std::string IMU240_PATH;
extern std::string IMU300_PATH;
extern std::string IMU360_PATH;
extern std::string IMU480_PATH;
extern std::string IMU600_PATH;

extern std::string GS_IMAGE_TOPIC;
extern std::string RS_IMAGE_TOPIC;
extern std::string IMU90_TOPIC;
extern std::string IMU120_TOPIC;
extern std::string IMU150_TOPIC;
extern std::string IMU180_TOPIC;
extern std::string IMU240_TOPIC;
extern std::string IMU300_TOPIC;
extern std::string IMU360_TOPIC;
extern std::string IMU480_TOPIC;
extern std::string IMU600_TOPIC;

extern std::string TOBAG_PATH;

void readParameters(ros::NodeHandle &n);
