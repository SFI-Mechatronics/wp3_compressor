#include <stdio.h>
#include <string.h>
#include <sstream>
#include <cstdlib>
#include <signal.h>

#include <ros/ros.h>

#include <Eigen/Eigen>

#include "wp3_compressor/defines.h"
#include "wp3_compressor/compressor.h"


void killHandler(int)
{
	ROS_INFO("%s","Shutdown request received.");
	ros::NodeHandle nh;
	ROS_INFO("%s","Terminating nodehandle.");
	nh.shutdown();
	ROS_INFO("%s","Terminating rosnode.");
	ros::shutdown();

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, _DEFAULTNODENAME, ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");

	signal(SIGINT, killHandler);
	signal(SIGTERM, killHandler);

	ros::Rate loopRate(_ROSRATE);

	if(!nh.hasParam("sensor_name"))
		ROS_ERROR("%s","Missing _sensor_name:=<name> parameter! Shutting down...");
	else if(!nh.hasParam("resolution"))
		ROS_ERROR("%s","Missing _resolution:=<resolution> parameter! Shutting down...");
	else {

		std::string sensorName;
		nh.getParam("sensor_name", sensorName);

		double resolution;
		nh.getParam("resolution", resolution);

		ros::Duration(1.0).sleep();

		Eigen::Vector4f minPT, maxPT;
		minPT << _MINX, _MINY, _MINZ, 1;
		maxPT << _MAXX, _MAXY, _MAXZ, 1;

		std::string outputTopic = "/" + sensorName + _TOPICOUT;
		std::string inputTopic = "/" + sensorName + _KINECTPOINTS;
		std::string localFrame = sensorName + _KINECTFRAME;
		std::string globalFrame = _GLOBALFRAME;

		wp3::CloudCompressor compressor(outputTopic, inputTopic, localFrame, globalFrame, resolution, _IFRAMERATE, minPT, maxPT, _STATISTICS);

		//		wp3::CloudDecompressor decompressor(sensorName, _TOPICOUT, false);

		std::cout << 3.0f * sizeof (float) << std::endl;
		while(ros::ok()){
			time_t start = clock();

			ros::spinOnce();
			clock_t end = clock();
			double time = (double) (end-start) / CLOCKS_PER_SEC * 1000.0;
			if(_STATISTICS)
				std::cout << time << std::endl;
			loopRate.sleep();
		}

	} // end else if has sensorname

	if(ros::ok()){
		nh.shutdown();
		ros::shutdown();
	}

	ROS_INFO("%s","Shutdown complete.");
	return 0;
}
