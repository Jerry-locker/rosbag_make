#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

extern std::string GS_CAM_PATH;
extern std::string RS_CAM_PATH;
extern std::string IMU_PATH;

extern std::string GS_IMAGE_TOPIC;
extern std::string RS_IMAGE_TOPIC;
extern std::string IMU_TOPIC;
extern std::string TOBAG_PATH;

void readParameters(ros::NodeHandle &n);
