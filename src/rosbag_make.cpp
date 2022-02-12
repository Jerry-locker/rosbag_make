#include <stdio.h>
#include <fstream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "parameters.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosbag_make");
  ros::NodeHandle n("~");
  readParameters(n);

  rosbag::Bag newbag;
  newbag.open(TOBAG_PATH, rosbag::bagmode::Write);

  ///save gsimage to bag
  std::string gs_cam_data_csv = GS_CAM_PATH + "/data.csv";
  std::fstream gs_cam_file(gs_cam_data_csv);
  //scan every line
  std::string gs_current_line_cam;
  while (std::getline(gs_cam_file, gs_current_line_cam))
  {
    static int gscnt = 0;
    gscnt++;
    if (gscnt == 1)
      continue;

    std::stringstream sline(gs_current_line_cam);
    std::string element;
    std::vector<std::string> vec;

    while (std::getline(sline, element, ','))
    {
      if (element.empty())
        continue;
      vec.push_back(element);
    }

    assert(vec.size() == 2);
    long timestamp = std::stol(vec[0]) + ros::TIME_MIN.toNSec();
    //std::string pic_path = GS_CAM_PATH + "/img/" + vec[1].substr(0, vec[1].size() - 1); //vec[1]尾部带有换行符
    std::string pic_path = GS_CAM_PATH + "/img/" + vec[1]; //vec[1]尾部带有换行符

    cv::Mat image = cv::imread(pic_path, cv::IMREAD_UNCHANGED); //不要改变通道数
    //std::cout << image.type() << std::endl;
    if (image.data == nullptr)
    {
      std::cout << "文件" << pic_path << "不存在\n";
      ros::shutdown();
    }
    std_msgs::Header header;
    header.stamp = ros::Time().fromNSec(timestamp);
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    //sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();

    newbag.write(GS_IMAGE_TOPIC, img_msg->header.stamp, img_msg);
  }

  ///save rsimage to bag
  std::string rs_cam_data_csv = RS_CAM_PATH + "/data.csv";
  std::fstream rs_cam_file(rs_cam_data_csv);
  //scan every line
  std::string rs_current_line_cam;
  while (std::getline(rs_cam_file, rs_current_line_cam))
  {
    static int gscnt = 0;
    gscnt++;
    if (gscnt == 1)
      continue;

    std::stringstream sline(rs_current_line_cam);
    std::string element;
    std::vector<std::string> vec;

    while (std::getline(sline, element, ','))
    {
      if (element.empty())
        continue;
      vec.push_back(element);
    }

    assert(vec.size() == 2);
    long timestamp = std::stol(vec[0]) + ros::TIME_MIN.toNSec();
    //std::string pic_path = RS_CAM_PATH + "/img/" + vec[1].substr(0, vec[1].size() - 1); //vec[1]尾部带有换行符
    std::string pic_path = RS_CAM_PATH + "/img/" + vec[1]; //vec[1]尾部带有换行符

    cv::Mat image = cv::imread(pic_path, cv::IMREAD_UNCHANGED); //不要改变通道数
    //std::cout << image.type() << std::endl;
    if (image.data == nullptr)
    {
      std::cout << "文件" << pic_path << "不存在\n";
      ros::shutdown();
    }
    std_msgs::Header header;
    header.stamp = ros::Time().fromNSec(timestamp);
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
    //sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();

    newbag.write(RS_IMAGE_TOPIC, img_msg->header.stamp, img_msg);
  }

  ///save imu to bag 90
  std::fstream imu_file90(IMU90_PATH);
  //scan every line
  std::string current_line_imu90;
  while (std::getline(imu_file90, current_line_imu90))
  {
    static int cnt90 = 0;
    cnt90++;
    if (cnt90 == 1)
      continue;

    std::stringstream sline(current_line_imu90);
    std::string element;
    std::vector<std::string> vec;

    while (std::getline(sline, element, ','))
    {
      if (element.empty())
        continue;
      vec.push_back(element);
    }

    assert(vec.size() == 7);
    long timestamp = std::stol(vec[0]) + ros::TIME_MIN.toNSec();

    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu); //即使是ros的智能指针也要new
    imu_msg->header.stamp = ros::Time().fromNSec(timestamp);
    imu_msg->angular_velocity.x = std::stof(vec[1]);
    imu_msg->angular_velocity.y = std::stof(vec[2]);
    imu_msg->angular_velocity.z = std::stof(vec[3]);
    imu_msg->linear_acceleration.x = std::stof(vec[4]);
    imu_msg->linear_acceleration.y = std::stof(vec[5]);
    imu_msg->linear_acceleration.z = std::stof(vec[6].substr(0, vec[6].size() - 1));

    newbag.write(IMU90_TOPIC, imu_msg->header.stamp, imu_msg);
  }

  ///save imu to bag 120
  std::fstream imu_file120(IMU120_PATH);
  //scan every line
  std::string current_line_imu120;
  while (std::getline(imu_file120, current_line_imu120))
  {
    static int cnt120 = 0;
    cnt120++;
    if (cnt120 == 1)
      continue;

    std::stringstream sline(current_line_imu120);
    std::string element;
    std::vector<std::string> vec;

    while (std::getline(sline, element, ','))
    {
      if (element.empty())
        continue;
      vec.push_back(element);
    }

    assert(vec.size() == 7);
    long timestamp = std::stol(vec[0]) + ros::TIME_MIN.toNSec();

    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu); //即使是ros的智能指针也要new
    imu_msg->header.stamp = ros::Time().fromNSec(timestamp);
    imu_msg->angular_velocity.x = std::stof(vec[1]);
    imu_msg->angular_velocity.y = std::stof(vec[2]);
    imu_msg->angular_velocity.z = std::stof(vec[3]);
    imu_msg->linear_acceleration.x = std::stof(vec[4]);
    imu_msg->linear_acceleration.y = std::stof(vec[5]);
    imu_msg->linear_acceleration.z = std::stof(vec[6].substr(0, vec[6].size() - 1));

    newbag.write(IMU120_TOPIC, imu_msg->header.stamp, imu_msg);
  }

  ///save imu to bag 150
  std::fstream imu_file150(IMU150_PATH);
  //scan every line
  std::string current_line_imu150;
  while (std::getline(imu_file150, current_line_imu150))
  {
    static int cnt150 = 0;
    cnt150++;
    if (cnt150 == 1)
      continue;

    std::stringstream sline(current_line_imu150);
    std::string element;
    std::vector<std::string> vec;

    while (std::getline(sline, element, ','))
    {
      if (element.empty())
        continue;
      vec.push_back(element);
    }

    assert(vec.size() == 7);
    long timestamp = std::stol(vec[0]) + ros::TIME_MIN.toNSec();

    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu); //即使是ros的智能指针也要new
    imu_msg->header.stamp = ros::Time().fromNSec(timestamp);
    imu_msg->angular_velocity.x = std::stof(vec[1]);
    imu_msg->angular_velocity.y = std::stof(vec[2]);
    imu_msg->angular_velocity.z = std::stof(vec[3]);
    imu_msg->linear_acceleration.x = std::stof(vec[4]);
    imu_msg->linear_acceleration.y = std::stof(vec[5]);
    imu_msg->linear_acceleration.z = std::stof(vec[6].substr(0, vec[6].size() - 1));

    newbag.write(IMU150_TOPIC, imu_msg->header.stamp, imu_msg);
  }

  ///save imu to bag 180
  std::fstream imu_file180(IMU180_PATH);
  //scan every line
  std::string current_line_imu180;
  while (std::getline(imu_file180, current_line_imu180))
  {
    static int cnt180 = 0;
    cnt180++;
    if (cnt180 == 1)
      continue;

    std::stringstream sline(current_line_imu180);
    std::string element;
    std::vector<std::string> vec;

    while (std::getline(sline, element, ','))
    {
      if (element.empty())
        continue;
      vec.push_back(element);
    }

    assert(vec.size() == 7);
    long timestamp = std::stol(vec[0]) + ros::TIME_MIN.toNSec();

    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu); //即使是ros的智能指针也要new
    imu_msg->header.stamp = ros::Time().fromNSec(timestamp);
    imu_msg->angular_velocity.x = std::stof(vec[1]);
    imu_msg->angular_velocity.y = std::stof(vec[2]);
    imu_msg->angular_velocity.z = std::stof(vec[3]);
    imu_msg->linear_acceleration.x = std::stof(vec[4]);
    imu_msg->linear_acceleration.y = std::stof(vec[5]);
    imu_msg->linear_acceleration.z = std::stof(vec[6].substr(0, vec[6].size() - 1));

    newbag.write(IMU180_TOPIC, imu_msg->header.stamp, imu_msg);
  }

  ///save imu to bag 240
  std::fstream imu_file240(IMU240_PATH);
  //scan every line
  std::string current_line_imu240;
  while (std::getline(imu_file240, current_line_imu240))
  {
    static int cnt240 = 0;
    cnt240++;
    if (cnt240 == 1)
      continue;

    std::stringstream sline(current_line_imu240);
    std::string element;
    std::vector<std::string> vec;

    while (std::getline(sline, element, ','))
    {
      if (element.empty())
        continue;
      vec.push_back(element);
    }

    assert(vec.size() == 7);
    long timestamp = std::stol(vec[0]) + ros::TIME_MIN.toNSec();

    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu); //即使是ros的智能指针也要new
    imu_msg->header.stamp = ros::Time().fromNSec(timestamp);
    imu_msg->angular_velocity.x = std::stof(vec[1]);
    imu_msg->angular_velocity.y = std::stof(vec[2]);
    imu_msg->angular_velocity.z = std::stof(vec[3]);
    imu_msg->linear_acceleration.x = std::stof(vec[4]);
    imu_msg->linear_acceleration.y = std::stof(vec[5]);
    imu_msg->linear_acceleration.z = std::stof(vec[6].substr(0, vec[6].size() - 1));

    newbag.write(IMU240_TOPIC, imu_msg->header.stamp, imu_msg);
  }

  ///save imu to bag 300
  std::fstream imu_file300(IMU300_PATH);
  //scan every line
  std::string current_line_imu300;
  while (std::getline(imu_file300, current_line_imu300))
  {
    static int cnt300 = 0;
    cnt300++;
    if (cnt300 == 1)
      continue;

    std::stringstream sline(current_line_imu300);
    std::string element;
    std::vector<std::string> vec;

    while (std::getline(sline, element, ','))
    {
      if (element.empty())
        continue;
      vec.push_back(element);
    }

    assert(vec.size() == 7);
    long timestamp = std::stol(vec[0]) + ros::TIME_MIN.toNSec();

    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu); //即使是ros的智能指针也要new
    imu_msg->header.stamp = ros::Time().fromNSec(timestamp);
    imu_msg->angular_velocity.x = std::stof(vec[1]);
    imu_msg->angular_velocity.y = std::stof(vec[2]);
    imu_msg->angular_velocity.z = std::stof(vec[3]);
    imu_msg->linear_acceleration.x = std::stof(vec[4]);
    imu_msg->linear_acceleration.y = std::stof(vec[5]);
    imu_msg->linear_acceleration.z = std::stof(vec[6].substr(0, vec[6].size() - 1));

    newbag.write(IMU300_TOPIC, imu_msg->header.stamp, imu_msg);
  }

  ///save imu to bag 360
  std::fstream imu_file360(IMU360_PATH);
  //scan every line
  std::string current_line_imu360;
  while (std::getline(imu_file360, current_line_imu360))
  {
    static int cnt360 = 0;
    cnt360++;
    if (cnt360 == 1)
      continue;

    std::stringstream sline(current_line_imu360);
    std::string element;
    std::vector<std::string> vec;

    while (std::getline(sline, element, ','))
    {
      if (element.empty())
        continue;
      vec.push_back(element);
    }

    assert(vec.size() == 7);
    long timestamp = std::stol(vec[0]) + ros::TIME_MIN.toNSec();

    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu); //即使是ros的智能指针也要new
    imu_msg->header.stamp = ros::Time().fromNSec(timestamp);
    imu_msg->angular_velocity.x = std::stof(vec[1]);
    imu_msg->angular_velocity.y = std::stof(vec[2]);
    imu_msg->angular_velocity.z = std::stof(vec[3]);
    imu_msg->linear_acceleration.x = std::stof(vec[4]);
    imu_msg->linear_acceleration.y = std::stof(vec[5]);
    imu_msg->linear_acceleration.z = std::stof(vec[6].substr(0, vec[6].size() - 1));

    newbag.write(IMU360_TOPIC, imu_msg->header.stamp, imu_msg);
  }

  ///save imu to bag 480
  std::fstream imu_file480(IMU480_PATH);
  //scan every line
  std::string current_line_imu480;
  while (std::getline(imu_file480, current_line_imu480))
  {
    static int cnt480 = 0;
    cnt480++;
    if (cnt480 == 1)
      continue;

    std::stringstream sline(current_line_imu480);
    std::string element;
    std::vector<std::string> vec;

    while (std::getline(sline, element, ','))
    {
      if (element.empty())
        continue;
      vec.push_back(element);
    }

    assert(vec.size() == 7);
    long timestamp = std::stol(vec[0]) + ros::TIME_MIN.toNSec();

    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu); //即使是ros的智能指针也要new
    imu_msg->header.stamp = ros::Time().fromNSec(timestamp);
    imu_msg->angular_velocity.x = std::stof(vec[1]);
    imu_msg->angular_velocity.y = std::stof(vec[2]);
    imu_msg->angular_velocity.z = std::stof(vec[3]);
    imu_msg->linear_acceleration.x = std::stof(vec[4]);
    imu_msg->linear_acceleration.y = std::stof(vec[5]);
    imu_msg->linear_acceleration.z = std::stof(vec[6].substr(0, vec[6].size() - 1));

    newbag.write(IMU480_TOPIC, imu_msg->header.stamp, imu_msg);
  }

  ///save imu to bag 600
  std::fstream imu_file600(IMU600_PATH);
  //scan every line
  std::string current_line_imu600;
  while (std::getline(imu_file600, current_line_imu600))
  {
    static int cnt600 = 0;
    cnt600++;
    if (cnt600 == 1)
      continue;

    std::stringstream sline(current_line_imu600);
    std::string element;
    std::vector<std::string> vec;

    while (std::getline(sline, element, ','))
    {
      if (element.empty())
        continue;
      vec.push_back(element);
    }

    assert(vec.size() == 7);
    long timestamp = std::stol(vec[0]) + ros::TIME_MIN.toNSec();

    sensor_msgs::ImuPtr imu_msg(new sensor_msgs::Imu); //即使是ros的智能指针也要new
    imu_msg->header.stamp = ros::Time().fromNSec(timestamp);
    imu_msg->angular_velocity.x = std::stof(vec[1]);
    imu_msg->angular_velocity.y = std::stof(vec[2]);
    imu_msg->angular_velocity.z = std::stof(vec[3]);
    imu_msg->linear_acceleration.x = std::stof(vec[4]);
    imu_msg->linear_acceleration.y = std::stof(vec[5]);
    imu_msg->linear_acceleration.z = std::stof(vec[6].substr(0, vec[6].size() - 1));

    newbag.write(IMU600_TOPIC, imu_msg->header.stamp, imu_msg);
  }


  newbag.close();

  std::cout << "[make bag ok]\n";

  return 0;
}
