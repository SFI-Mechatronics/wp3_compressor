/*
 * decompressor.h
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#ifndef DECOMPRESSOR_H_
#define DECOMPRESSOR_H_

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <cstdlib>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include "octree_decompression.h"

// Defines
#define _GLOBALFRAME "world"


typedef pcl::PointXYZI PointType_out;
typedef pcl::PointCloud<PointType_out> PointCloudXYZI;

typedef wp3::PointCloudDecompression Decompressor;

namespace wp3 {

class CloudDecompressor
{
public:
	// Constructor
	CloudDecompressor(std::string outputCloudTopic, std::string inputMsgTopic, const float intensityLimit, const bool showStatistics);

	// Deconstrucor
	~CloudDecompressor();

	// Callback for PointCloudXYZ subscriber
	void roscallback(const std_msgs::String::ConstPtr& msg);

private:

	// ROS variables
	ros::NodeHandle nh_;
	ros::Subscriber sub_;
	ros::Publisher pub_;

	// Pointers to temporary point clouds
	PointCloudXYZI::Ptr decompressedCloud_;
	PointCloudXYZI::Ptr outputCloud_;

	Decompressor pointCloudDecoder_;

	// Passthrough filter
	float intensityLimit_;
	pcl::PassThrough<PointType_out> ptfilter_; // Initializing with true will allow us to extract the removed indices

	// Logging
	bool showStatistics_;
	std::string logFile_;
	std::ofstream logStream_;

};

}


#endif /* DECOMPRESSOR_H_ */
