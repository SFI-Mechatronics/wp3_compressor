/*
 * compressor.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#include "wp3_compressor/compressor.h"

namespace wp3 {

// Constructor
CloudCompressor::CloudCompressor(std::string outputMsgTopic, std::string globalFrame, std::string localFrame, double octreeResolution,
                                 unsigned int iFrameRate, Eigen::Vector4f minPT, Eigen::Vector4f maxPT, bool showStatistics) :
  transformedCloud(new PointCloud()),
  croppedCloud(new PointCloud ()),
  octreeResolution(octreeResolution),
  globalFrame(globalFrame),
  localFrame(localFrame),
  showStatistics(showStatistics),
  dataReceived(false),
  logFile("/home/nvidia/compressorlog.txt"),
  pointCloudEncoder(showStatistics, octreeResolution, minPT[0], minPT[1], minPT[2], maxPT[0], maxPT[1], maxPT[2], iFrameRate)
{
  if(showStatistics){
    logStream.open(logFile.c_str());
    logStream << "Opoints\tOsize\tCpoint\tCsize\tTime" << std::endl;
  }

  // Setup box crop filter

  crop.setMin(minPT);
  crop.setMax(maxPT);

  pub = nh.advertise<std_msgs::String>(outputMsgTopic, 1);
}

CloudCompressor::~CloudCompressor(){
  if(showStatistics)
    logStream.close();
}

void CloudCompressor::setTransform(const tf::StampedTransform &value)
{
  transform = value;
}

std::string CloudCompressor::getGlobalFrame() const
{
  return globalFrame;
}

std::string CloudCompressor::getLocalFrame() const
{
  return localFrame;
}

void CloudCompressor::setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &value)
{
  pcl::copyPointCloud(*value, inputCloud);
}

void CloudCompressor::setInputCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &value)
{
  pcl::copyPointCloud(*value, inputCloud);
}

void CloudCompressor::setDataReceived(bool value)
{
  dataReceived = value;
}


// Callback for ROS subscriber
void CloudCompressor::Publish(){

  if(dataReceived){
    dataReceived = false;

    time_t start = clock();

    // Transform the point cloud
    pcl_ros::transformPointCloud(*transformedCloud, inputCloud, transform);

    // Crop the point cloud
    crop.setInputCloud(transformedCloud);
    crop.filter(*croppedCloud);

    // Stream for storing serialized compressed point cloud
    std::stringstream compressedData;

    // Encode point cloud to stream
    pointCloudEncoder.encodePointCloud (croppedCloud, compressedData);

    clock_t end = clock();
    double time = (double) (end-start) / CLOCKS_PER_SEC * 1000.0;

    // Publish the encoded stream
    std_msgs::String msg;
    msg.data = compressedData.str();
    pub.publish(msg);

    if (showStatistics)
    {
      float input_size = static_cast<float> (inputCloud.size());
      float cropped_size = static_cast<float> (croppedCloud->size());

      //				PCL_INFO ("*** POINTCLOUD FILTERING ***\n");
      //
      //				PCL_INFO ("Number of points in original cloud: %.0f\n", input_size);
      //				PCL_INFO ("Size original point cloud: %.0f kBytes\n", static_cast<float> ((input_size) * (3.0f * sizeof (float))) / 1024.0f);
      //				PCL_INFO ("Number of points in cropped cloud: %.0f\n", cropped_size);
      //				PCL_INFO ("Size cropped point cloud: %f kBytes\n\n", static_cast<float> ((cropped_size) * (3.0f * sizeof (float))) / 1024.0f);

      logStream << input_size << "\t" << static_cast<float> ((input_size) * (4.0f * sizeof (float))) / 1024.0f << "\t";
      logStream << cropped_size << "\t" << static_cast<float> ((cropped_size) * (4.0f * sizeof (float))) / 1024.0f << "\t";
      logStream << time << std::endl;
    }
  }

}

}
