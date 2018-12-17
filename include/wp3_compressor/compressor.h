/*
 * compressor.h
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#ifndef COMPRESSOR_H_
#define COMPRESSOR_H_

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <Eigen/Eigen>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

#include "octree_compression.h"


namespace wp3 {

class CloudCompressor
{
  typedef pcl::PointXYZI PointType;
  typedef pcl::PointCloud<PointType> PointCloud;
  typedef wp3::PointCloudCompression Compressor;

public:
  // Constructor
  CloudCompressor(std::string outputMsgTopic, std::string globalFrame, std::string localFrame, double octreeResolution,
                  unsigned int iFrameRate, Eigen::Vector4f minPT, Eigen::Vector4f maxPT, bool showStatistics);

  // Deconstrucor
  ~CloudCompressor();

  // Process and publish compressed cloud.
  void Publish();

  void setTransform(const tf::StampedTransform & value);

  std::string getGlobalFrame() const;

  std::string getLocalFrame() const;

  void setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & value);
  void setInputCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & value);

  void setDataReceived(bool value);

private:

  // ROS variables
  std::string globalFrame;
  std::string localFrame;

  ros::NodeHandle nh;
  ros::Publisher pub;

  tf::StampedTransform transform;

  PointCloud inputCloud;

  // Pointers to temporary point clouds
  PointCloud::Ptr transformedCloud;
  PointCloud::Ptr croppedCloud;

  // Compression setup
  double octreeResolution;
  float normDist;
  float voxValue;
  Compressor pointCloudEncoder;

  // Box crop filter
  pcl::CropBox<PointType> crop;

  bool dataReceived;

  // Logging
  bool showStatistics;
  std::string logFile;
  std::ofstream logStream;
};

}


#endif /* COMPRESSOR_H_ */
