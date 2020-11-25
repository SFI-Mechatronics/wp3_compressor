/*
 * compressor.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#include "wp3_compressor/compressor.h"
#include "wp3_compressor/comp_msg.h"
#include <math.h>

namespace wp3 {

// Constructor
CloudCompressor::CloudCompressor(std::string outputMsgTopic, std::string globalFrame, std::string localFrame, double octreeResolution,
                                 unsigned int iFrameRate, Eigen::Vector4f minPT, Eigen::Vector4f maxPT, bool showStatistics) :
  transformedCloud(new PointCloud()),
  croppedCloud(new PointCloud ()),
  tfListener(),
  octreeResolution(octreeResolution),
  globalFrame(globalFrame),
  localFrame(localFrame),
  showStatistics(showStatistics),
  logFile("/home/nvidia/compressorlog.txt"),
  pointCloudEncoder(minPT, maxPT, octreeResolution, iFrameRate, showStatistics)
{
  if(showStatistics){
    logStream.open(logFile.c_str());
    logStream << "Opoints\tOsize\tCpoint\tCsize\tTime" << std::endl;
  }

  // Setup box crop filter
  crop.setMin(minPT);
  crop.setMax(maxPT);

  pub = nh.advertise<wp3_compressor::comp_msg>(outputMsgTopic, 1);

  normDist = (octreeResolution * 360.0 * 212.0)/(2.0 * 30.0 * 3.14159265);
  double pi = 3.14159265;

  voxValue = ((tan(30.0/180.0*pi)*tan(35.3/180.0*pi)*4.0))/(octreeResolution*octreeResolution*217088.0);

}

CloudCompressor::~CloudCompressor(){
  if(showStatistics)
    logStream.close();
}

void CloudCompressor::compressCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & value)
{
  // pcl::copyPointCloud(*value, inputCloud);
  inputCloud.points.resize(value->size());
  int j = 0;
  for (size_t i = 0; i < value->size(); i++) {
    if(std::isfinite(value->points[i].z)){
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
  inputCloud.header.stamp = value->header.stamp;
  inputCloud.header.seq = value->header.seq;
  inputCloud.header.frame_id = value->header.frame_id;

  Publish();
}

void CloudCompressor::compressCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr & value)
{
  //pcl::copyPointCloud(*value, inputCloud);
  inputCloud = *value;

  Publish();
}


// Callback for ROS subscriber
void CloudCompressor::Publish(){

  // Get transformation published by master
  tf::StampedTransform transform;
  try{
    tfListener.lookupTransform(globalFrame, localFrame, ros::Time(0), transform);
  }
  catch (tf::TransformException & ex) {
    ROS_ERROR("Local TF: %s",ex.what());
    return;
  }

  // Transform the point cloud
  pcl_ros::transformPointCloud(inputCloud, *transformedCloud, transform);

  // Crop the point cloud only if crop box is defined
  if (crop.getMin() == crop.getMax()){
    croppedCloud = transformedCloud;
  }
  else {
    crop.setInputCloud(transformedCloud);
    crop.filter(*croppedCloud);
  }

  // Stream for storing serialized compressed point cloud
  std::stringstream compressedData;

  // Encode point cloud to stream
  pointCloudEncoder.encodePointCloud(croppedCloud, compressedData);

  // Publish the encoded stream if not empty
  if(compressedData.rdbuf()->in_avail() != 0){
    wp3_compressor::comp_msg msg;
    msg.data = compressedData.str();
    msg.header.seq = inputCloud.header.seq;
    pcl_conversions::fromPCL(inputCloud.header.stamp, msg.header.stamp);
    msg.header.frame_id = croppedCloud->header.frame_id;

    pub.publish(msg);
  }
  if (showStatistics)
  {
    float input_size = static_cast<float> (inputCloud.size());
    float cropped_size = static_cast<float> (croppedCloud->size());

    logStream << input_size << "\t" << static_cast<float> ((input_size) * (4.0f * sizeof (float))) / 1024.0f << "\t";
    logStream << cropped_size << "\t" << static_cast<float> ((cropped_size) * (4.0f * sizeof (float))) / 1024.0f << "\t";

  }

}

}
