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
  // Initialize ROS
  ros::init(argc, argv, _DEFAULTNODENAME, ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");

  signal(SIGINT, killHandler);
  signal(SIGTERM, killHandler);

  ros::Rate loopRate(_ROSRATE);


  if(!nh.hasParam("sensor_name"))
    ROS_ERROR("%s","Missing _sensor_name:=<name> parameter! Shutting down...");
  else if(!nh.hasParam("resolution"))
    ROS_ERROR("%s","Missing _resolution:=<resolution> parameter! Shutting down...");
  else if(!nh.hasParam("sensor_type"))
    ROS_ERROR("%s","Missing _sensor_type:=<0: Kinect, 1: Velodyne> parameter! Shutting down...");
  else {

    std::string outputTopic;
    std::string inputTopic;
    std::string localFrame;
    std::string globalFrame;

    std::string sensorName;
    double resolution;
    int sensorType;
    nh.getParam("sensor_name", sensorName);
    nh.getParam("resolution", resolution);
    nh.getParam("sensor_type", sensorType);

    switch(sensorType) {
    case 0: // Kinect
      ROS_INFO("Starting node using KINECT V2");
      outputTopic = "/" + sensorName + _TOPICOUT + "kinect_comp";
      inputTopic = "/" + sensorName + _KINECTPOINTS;
      localFrame = sensorName + _KINECTFRAME;
      globalFrame = _GLOBALFRAME;
      break;

    case 1: // Velodyne
      ROS_INFO("Starting node using VELODYNE");
      outputTopic = "/" + sensorName + _TOPICOUT + "velodyne_comp";
      inputTopic = "/velodyne_points";
      localFrame = "velodyne";
      globalFrame = _GLOBALFRAME;
      break;

    } // End switch(sensorType)


    ros::Duration(1.0).sleep();

    Eigen::Vector4f minPT, maxPT;
    minPT << _MINX, _MINY, _MINZ, 1;
    maxPT << _MAXX, _MAXY, _MAXZ, 1;


    wp3::CloudCompressor compressor(outputTopic, inputTopic, localFrame, globalFrame, resolution, _IFRAMERATE, minPT, maxPT, _STATISTICS);


    while(ros::ok()){
#if _STATISTICS
      time_t start = clock();
#endif

      ros::spinOnce();

#if _STATISTICS
      clock_t end = clock();
      double time = (double) (end-start) / CLOCKS_PER_SEC * 1000.0;
      std::cout << "ROS Cycle time: " << time << " ms" << std::endl;
#endif

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
