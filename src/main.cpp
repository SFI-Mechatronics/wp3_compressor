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

// Callback for ROS subscriber
void roscallbackXYZ(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud, wp3::CloudCompressor * compressor,
                    tf::TransformListener * tfListener){

  // Get transformation published by master
  tf::StampedTransform transform;
  try{
    tfListener->lookupTransform(compressor->getGlobalFrame(), compressor->getLocalFrame(), ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("Local TF: %s",ex.what());
    return;
  }
  compressor->setTransform(transform);
  compressor->setInputCloud(cloud);
  compressor->setDataReceived(true);
}

void roscallbackXYZI(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, wp3::CloudCompressor * compressor,
                     tf::TransformListener * tfListener){

  // Get transformation published by master
  tf::StampedTransform transform;
  try{
    tfListener->lookupTransform(compressor->getGlobalFrame(), compressor->getLocalFrame(), ros::Time(0), transform);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("Local TF: %s",ex.what());
    return;
  }

  compressor->setTransform(transform);
  compressor->setInputCloud(cloud);
  compressor->setDataReceived(true);
}

int main(int argc, char **argv)
{
  // Initialize ROS
  ros::init(argc, argv, _DEFAULTNODENAME, ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");

  signal(SIGINT, killHandler);
  signal(SIGTERM, killHandler);

  ros::Rate loopRate(_ROSRATE);

  if(!nh.hasParam("resolution"))
    ROS_ERROR("%s","Missing _resolution:=<resolution> parameter! Shutting down...");
  else if(!nh.hasParam("input_topic"))
    ROS_ERROR("%s","Missing _input_topic parameter! Shutting down...");
  else if(!nh.hasParam("input_type"))
    ROS_ERROR("%s","Missing _input_type:=<0 = XYZ, 1 = XYZI, 2 = XYXRGB> parameter! Shutting down...");
  else if(!nh.hasParam("output_topic"))
    ROS_ERROR("%s","Missing _output_topic parameter! Shutting down...");
  else if(!nh.hasParam("local_frame"))
    ROS_ERROR("%s","Missing _local_frame parameter! Shutting down...");
  else if(!nh.hasParam("global_frame"))
    ROS_ERROR("%s","Missing _global_frame parameter! Shutting down...");
  else {

    std::string outputTopic;
    std::string inputTopic;
    int inputType;
    std::string localFrame;
    std::string globalFrame;
    double resolution;

    nh.getParam("resolution", resolution);
    nh.getParam("input_topic", inputTopic);
    nh.getParam("input_type", inputType);
    nh.getParam("output_topic", outputTopic);
    nh.getParam("local_frame", localFrame);
    nh.getParam("global_frame", globalFrame);

    ROS_INFO("%s", "Starting node.");
    //      inputTopic = "/" + baseName + "/" + inputTopic;
    //      outputTopic = "/" + basseName + _KINECTPOINTS;
    //      velodyneTopic = "/velodyne_points";

    //      kinectFrame = sensorName + _KINECTFRAME;
    //      velodyneFrame = "velodyne";
    //      globalFrame = _GLOBALFRAME;

    //    case 1: // Velodyne
    //      ROS_INFO("Starting node using VELODYNE");
    //      outputTopic = "/" + sensorName + _TOPICOUT + "velodyne_comp";
    //      inputTopic = "/velodyne_points";
    //      localFrame = "velodyne";
    //      globalFrame = _GLOBALFRAME;
    //      break;

    //    } // End switch(sensorType)


    ros::Duration(1.0).sleep();

    Eigen::Vector4f minPT, maxPT;
    minPT << _MINX, _MINY, _MINZ, 1;
    maxPT << _MAXX, _MAXY, _MAXZ, 1;

    wp3::CloudCompressor compressor(outputTopic, globalFrame, localFrame, resolution, _IFRAMERATE, minPT, maxPT, _STATISTICS);

    tf::TransformListener tfListener;
    ros::Subscriber sub;
    switch (inputType){
    case 0:
      sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >(inputTopic, 1, boost::bind(&roscallbackXYZ, _1, &compressor, &tfListener));
      break;
    case 1:
      sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZI> >(inputTopic, 1, boost::bind(&roscallbackXYZI, _1, &compressor, &tfListener));
      break;
    default:
      ROS_ERROR("%s","Incorrect input_type used!");
      break;
    }


    while(ros::ok()){
#if _STATISTICS
      time_t start = clock();
#endif

      ros::spinOnce();
      compressor.Publish();


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
