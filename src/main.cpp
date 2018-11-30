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
void roscallback(const sensor_msgs::PointCloud2ConstPtr &cloud1, const sensor_msgs::PointCloud2ConstPtr &cloud2, wp3::CloudCompressor * compressor){
  tf::TransformListener tfListener;

  // Get transformations published by master
  tf::StampedTransform transform1;
  try{
    tfListener.lookupTransform(compressor->getGlobalFrame(), compressor->getKinectFrame(), ros::Time(0), transform1);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    return;
  }
  compressor->setKinectTF(transform1);

  tf::StampedTransform transform2;
  try{
    tfListener.lookupTransform(compressor->getGlobalFrame(), compressor->getVelodyneFrame(), ros::Time(0), transform2);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    return;
  }
  compressor->setVelodyneTF(transform2);

  compressor->setKinectCloudPC2(cloud1);
  compressor->setVelodyneCloudPC2(cloud2);
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
//  else if(!nh.hasParam("sensor_type"))
//    ROS_ERROR("%s","Missing _sensor_type:=<0: Kinect, 1: Velodyne> parameter! Shutting down...");
  else {

    std::string outputTopic;
    std::string kinectTopic, velodyneTopic;
    std::string kinectFrame, velodyneFrame;
    std::string globalFrame;

    std::string sensorName;
    double resolution;
//    int sensorType;
    nh.getParam("sensor_name", sensorName);
    nh.getParam("resolution", resolution);
//    nh.getParam("sensor_type", sensorType);

      ROS_INFO("Starting node using KINECT V2 and VELODYNE");
      outputTopic = "/" + sensorName + _TOPICOUT + "kinect_comp";
      kinectTopic = "/" + sensorName + _KINECTPOINTS;
      velodyneTopic = "/velodyne_points";

      kinectFrame = sensorName + _KINECTFRAME;
      velodyneFrame = "velodyne";
      globalFrame = _GLOBALFRAME;

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

    wp3::CloudCompressor compressor(outputTopic, globalFrame, kinectFrame, velodyneFrame, resolution, _IFRAMERATE, minPT, maxPT, _STATISTICS);

    message_filters::Subscriber<sensor_msgs::PointCloud2> kinect_sub(nh, kinectTopic, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> velodyne_sub(nh, velodyneTopic, 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> SyncPolityT;

    message_filters::Synchronizer<SyncPolityT> sync(SyncPolityT(4), kinect_sub, velodyne_sub);
    sync.setInterMessageLowerBound(0,ros::Duration(loopRate));
    sync.setInterMessageLowerBound(1,ros::Duration(loopRate));

    sync.registerCallback(boost::bind(&roscallback, _1, _2, &compressor));


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
