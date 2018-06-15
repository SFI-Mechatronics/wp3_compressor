/*
 * compressor.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#include "wp3_compressor/compressor.h"

namespace wp3 {

// Constructor
CloudCompressor::CloudCompressor(std::string outputMsgTopic, std::string inputCloudTopic, std::string rosTFLocalFrame, std::string rosTFGlobalFrame,
		double octreeResolution, unsigned int iFrameRate, Eigen::Vector4f minPT, Eigen::Vector4f maxPT, bool showStatistics) :
										 transformedCloud_(new PointCloud()),
										 croppedCloud_(new PointCloud ()),
										 octreeResolution_(octreeResolution),
										 rosTFLocalFrame_(rosTFLocalFrame),
										 rosTFGlobalFrame_(rosTFGlobalFrame),
										 showStatistics_(showStatistics),
										 logFile_("/home/nvidia/compressorlog.txt"),
										 pointCloudEncoder_(showStatistics, octreeResolution, minPT[0], minPT[1], minPT[2], maxPT[0], maxPT[1], maxPT[2], iFrameRate)
{
	if(showStatistics_){
		logStream_.open(logFile_.c_str());
		logStream_ << "Opoints\tOsize\tCpoint\tCsize\tTime" << std::endl;
	}

	// Setup box crop filter

	crop_.setMin(minPT);
	crop_.setMax(maxPT);

	sub_ = nh_.subscribe<PointCloud>(inputCloudTopic, 1, &wp3::CloudCompressor::roscallback, this);
	pub_ = nh_.advertise<std_msgs::String>(outputMsgTopic, 1);
}

CloudCompressor::~CloudCompressor(){
	if(showStatistics_)
		logStream_.close();
}

// Callback for ROS subscriber
void CloudCompressor::roscallback(const PointCloud::ConstPtr &cloud){

	// Get transformation published by master
	tf::StampedTransform transform;
	try{
		tfListener_.lookupTransform(rosTFGlobalFrame_, rosTFLocalFrame_, ros::Time(0), transform);
	}
	catch (tf::TransformException &ex) {
		ROS_ERROR("%s",ex.what());
		return;
	}

	time_t start = clock();

	// Transform the point cloud
	pcl_ros::transformPointCloud(*cloud, *transformedCloud_, transform);

	// Crop the point cloud
	crop_.setInputCloud(transformedCloud_);
	crop_.filter(*croppedCloud_);

	// Stream for storing serialized compressed point cloud
	std::stringstream compressedData;

	// Encode point cloud to stream
	pointCloudEncoder_.encodePointCloud (croppedCloud_, compressedData);

	clock_t end = clock();
	double time = (double) (end-start) / CLOCKS_PER_SEC * 1000.0;

	// Publish the encoded stream
	std_msgs::String msg;
	msg.data = compressedData.str();
	pub_.publish(msg);

	if (showStatistics_)
	{
		float input_size = static_cast<float> (cloud->size());
		float cropped_size = static_cast<float> (croppedCloud_->size());

		//				PCL_INFO ("*** POINTCLOUD FILTERING ***\n");
		//
		//				PCL_INFO ("Number of points in original cloud: %.0f\n", input_size);
		//				PCL_INFO ("Size original point cloud: %.0f kBytes\n", static_cast<float> ((input_size) * (3.0f * sizeof (float))) / 1024.0f);
		//				PCL_INFO ("Number of points in cropped cloud: %.0f\n", cropped_size);
		//				PCL_INFO ("Size cropped point cloud: %f kBytes\n\n", static_cast<float> ((cropped_size) * (3.0f * sizeof (float))) / 1024.0f);

		logStream_ << input_size << "\t" << static_cast<float> ((input_size) * (3.0f * sizeof (float))) / 1024.0f << "\t";
		logStream_ << cropped_size << "\t" << static_cast<float> ((cropped_size) * (3.0f * sizeof (float))) / 1024.0f << "\t";
		logStream_ << time << std::endl;
	}


}


}
