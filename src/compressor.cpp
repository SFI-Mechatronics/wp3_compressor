/*
 * compressor.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#include "wp3_compressor/compressor.h"

namespace wp3 {

// Constructor
template <typename PointT>
CloudCompressor<PointT>::CloudCompressor(std::string outputMsgTopic, std::string globalFrame, std::string kinectFrame, std::string velodyneFrame,
                                 double octreeResolution, unsigned int iFrameRate, Eigen::Vector4f minPT, Eigen::Vector4f maxPT, bool showStatistics) :
  transformedCloud(new PointCloud()),
  croppedCloud(new PointCloud ()),
  octreeResolution(octreeResolution),
  globalFrame(globalFrame),
  kinectFrame(kinectFrame),
  velodyneFrame(velodyneFrame),
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

template <typename PointT>
CloudCompressor<PointT>::~CloudCompressor(){
  if(showStatistics)
    logStream.close();
}

template <typename PointT>
void CloudCompressor<PointT>::setKinectTF(const tf::StampedTransform &value)
{
  kinectTF = value;
}

template <typename PointT>
void CloudCompressor<PointT>::setVelodyneTF(const tf::StampedTransform &value)
{
  velodyneTF = value;
}

template <typename PointT>
std::string CloudCompressor<PointT>::getGlobalFrame() const
{
  return globalFrame;
}

template <typename PointT>
std::string CloudCompressor<PointT>::getKinectFrame() const
{
  return kinectFrame;
}

template <typename PointT>
std::string CloudCompressor<PointT>::getVelodyneFrame() const
{
  return velodyneFrame;
}

template <typename PointT>
void CloudCompressor<PointT>::setKinectCloud(const PointCloud &value)
{
  kinectCloud = value;
}

template <typename PointT>
void CloudCompressor<PointT>::setVelodyneCloud(const PointCloud &value)
{
  velodyneCloud = value;
}

template <typename PointT>
void CloudCompressor<PointT>::setKinectCloudPC2(const sensor_msgs::PointCloud2ConstPtr &value)
{
  pcl::fromROSMsg(*value,kinectCloud);
}

template <typename PointT>
void CloudCompressor<PointT>::setVelodyneCloudPC2(const sensor_msgs::PointCloud2ConstPtr &value)
{
  pcl::fromROSMsg(*value,velodyneCloud);
}

template <typename PointT>
void CloudCompressor<PointT>::setDataReceived(bool value)
{
  dataReceived = value;
}


// Callback for ROS subscriber
template <typename PointT>
void CloudCompressor<PointT>::Publish(){

  if(dataReceived){
    dataReceived = false;

    time_t start = clock();

    // Transform the point cloud
    pcl_ros::transformPointCloud(kinectCloud, kinectCloud, kinectTF);
    pcl_ros::transformPointCloud(velodyneCloud, velodyneCloud, velodyneTF);

    *transformedCloud = kinectCloud + velodyneCloud;

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
      float input_size = static_cast<float> (kinectCloud.size() + velodyneCloud.size());
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
  template class CloudCompressor<pcl::PointXYZ>;
  template class CloudCompressor<pcl::PointXYZI>;

}
