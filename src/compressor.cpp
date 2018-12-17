/*
 * compressor.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#include "wp3_compressor/compressor.h"
#include <math.h>

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

  normDist = (octreeResolution * 360.0 * 212.0)/(2.0 * 30.0 * 3.14159265);
  double pi = 3.14159265;

  voxValue = ((tan(30.0/180.0*pi)*tan(35.3/180.0*pi)*4.0))/(octreeResolution*octreeResolution*217088.0);
  ROS_ERROR("%f",voxValue);
}

CloudCompressor::~CloudCompressor(){
  if(showStatistics)
    logStream.close();
}

void CloudCompressor::setTransform(const tf::StampedTransform & value)
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

void CloudCompressor::setInputCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & value)
{
  // pcl::copyPointCloud(*value, inputCloud);
  inputCloud.points.resize(value->size());
  int j = 0;
  for (size_t i = 0; i < value->size(); i++) {
    if(pcl_isfinite(value->points[i].z)){
      inputCloud.points[j].x = value->points[i].x;
      inputCloud.points[j].y = value->points[i].y;
      inputCloud.points[j].z = value->points[i].z;
      inputCloud.points[j].intensity = std::min<float>((value->points[i].z * value->points[i].z * voxValue),1.0f) * 255.0f;

      //inputCloud.points[j].intensity = std::min((value->points[i].z / normDist),1.0f) * 127.0;
      j++;
    }
  }
  inputCloud.points.resize(j);
  inputCloud.height = 1;
  inputCloud.width  = static_cast<uint32_t>(j);
  inputCloud.is_dense = true;
}

void CloudCompressor::setInputCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & value)
{
  //pcl::copyPointCloud(*value, inputCloud);
  inputCloud = *value;
}

void CloudCompressor::setDataReceived(bool value)
{
  dataReceived = value;
}


// Callback for ROS subscriber
void CloudCompressor::Publish(){

  if(dataReceived){
    dataReceived = false;

    clock_t start = clock();

    // Transform the point cloud
    pcl_ros::transformPointCloud(inputCloud, *transformedCloud, transform);

    // Crop the point cloud
    crop.setInputCloud(transformedCloud);
    crop.filter(*croppedCloud);

    // Stream for storing serialized compressed point cloud
    std::stringstream compressedData;

    // Encode point cloud to stream
    pointCloudEncoder.encodePointCloud(croppedCloud, compressedData);

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
