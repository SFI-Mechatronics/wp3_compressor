/*
 * decompressor.cpp
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#include "wp3_decompressor/decompressor.h"

namespace wp3 {

// Constructor
CloudDecompressor::CloudDecompressor(std::string outputCloudTopic, std::string inputMsgTopic, const float intensityLimit, const bool showStatistics) :
								 decompressedCloud_(new PointCloudXYZI ()),
								 outputCloud_(new PointCloudXYZI ()),
								 ptfilter_(false),
								 intensityLimit_(intensityLimit),
								 showStatistics_(showStatistics),
								 logFile_("/home/sfi/decompressorlog.txt"),
								 pointCloudDecoder_(showStatistics)
{
	if(showStatistics_){
		logStream_.open(logFile_.c_str());
		logStream_ << "Time" << std::endl;
	}

	sub_ = nh_.subscribe<std_msgs::String>(inputMsgTopic, 1, &wp3::CloudDecompressor::roscallback, this);
	pub_ = nh_.advertise<PointCloudXYZI>(outputCloudTopic, 1);
}

CloudDecompressor::~CloudDecompressor(){
	if(showStatistics_)
		logStream_.close();
}

// Callback for ROS subscriber
void CloudDecompressor::roscallback(const std_msgs::String::ConstPtr& msg){

	time_t start = clock();

	// Stream for storing serialized compressed point cloud
	std::stringstream compressedData;

	// Retreive data from message
	compressedData << msg->data;

	// Decode stream to point cloud
	pointCloudDecoder_.decodePointCloud (compressedData, decompressedCloud_);

	// Filter point cloud based on intensity
	ptfilter_.setInputCloud (decompressedCloud_);
	ptfilter_.setFilterFieldName ("intensity");
	ptfilter_.setFilterLimits (intensityLimit_, FLT_MAX);
	ptfilter_.filter (*outputCloud_);

	clock_t end = clock();
	double time = (double) (end-start) / CLOCKS_PER_SEC * 1000.0;

	// Publish the decompressed cloud
	outputCloud_->header.frame_id = _GLOBALFRAME;
	pub_.publish(outputCloud_);

	if (showStatistics_)
	{
		logStream_ << time << std::endl;
	}

}

}
