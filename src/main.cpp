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
template<typename PT>
void roscallback(const typename pcl::PointCloud<PT>::ConstPtr & cloud, wp3::CloudCompressor * compressor,
                    tf::TransformListener * tfListener){

  // Get transformation published by master
  tf::StampedTransform transform;
  if (compressor->getGlobalFrame() == compressor->getLocalFrame())
      transform.setIdentity();
      else  {
        try  {
        tfListener->lookupTransform(compressor->getGlobalFrame(), compressor->getLocalFrame(), ros::Time(0), transform);
        }
        catch (tf::TransformException & ex) {
        ROS_ERROR("Local TF: %s",ex.what());
        return;
        }
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
    float minx, maxx, miny, maxy, minz, maxz;
    bool crop = false;

    nh.getParam("resolution", resolution);
    nh.getParam("input_topic", inputTopic);
    nh.getParam("input_type", inputType);
    nh.getParam("output_topic", outputTopic);
    nh.getParam("local_frame", localFrame);
    nh.getParam("global_frame", globalFrame);

    if(nh.hasParam("crop"))
      nh.getParam("crop", crop);

    if(nh.hasParam("min_x"))
      nh.getParam("min_x", minx);
    else      minx = _MINX;
    if(nh.hasParam("max_x"))
      nh.getParam("max_x", maxx);
    else      maxx = _MAXX;

    if(nh.hasParam("min_y"))
      nh.getParam("min_y", miny);
    else      miny = _MINY;
    if(nh.hasParam("max_y"))
      nh.getParam("max_y", maxy);
    else      maxy = _MAXY;

    if(nh.hasParam("min_z"))
      nh.getParam("min_z", minz);
    else      minz = _MINZ;
    if(nh.hasParam("max_z"))
      nh.getParam("max_z", maxz);
    else      maxz = _MAXZ;

    ROS_INFO("Initializing with the following parameters:");
    ROS_INFO("Resolution: %.2f m", resolution);
    ROS_INFO("Input topic: %s", inputTopic.c_str());
    ROS_INFO("Input topic type: %d (0 = XYZ, 1 = XYZI, 2 = XYXRGB)", inputType);
    ROS_INFO("Output topic: %s", outputTopic.c_str());

    ROS_INFO("Local frame: %s", localFrame.c_str());
    ROS_INFO("Global frame: %s", globalFrame.c_str());

    ROS_INFO("Crop Box: (%.2f , %.2f , %.2f) -> (%.2f , %.2f , %.2f)", minx, miny, minz, maxx, maxy, maxz);


    Eigen::Vector4f minPT, maxPT;
    if(crop){
      minPT << minx, miny, minz, 1;
      maxPT << maxx, maxy, maxz, 1;
    }
    else{
      minPT.setZero();
      maxPT.setZero();
    }


    wp3::CloudCompressor compressor(outputTopic, globalFrame, localFrame, resolution, _IFRAMERATE, minPT, maxPT, _STATISTICS);

    tf::TransformListener tfListener;
    ros::Subscriber sub;
    switch (inputType){
    case 0:
      sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ> >(inputTopic, 1, boost::bind(&roscallback<pcl::PointXYZ>, _1, &compressor, &tfListener));
      break;
    case 1:
      sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZI> >(inputTopic, 1, boost::bind(&roscallback<pcl::PointXYZI>, _1, &compressor, &tfListener));
      break;
    default:
      ROS_ERROR("%s","Incorrect input_type used!");
      break;
    }

    ROS_INFO("%s", "Starting node.");
    // Wait for TF listener to register TFs.
    ros::Duration(1.0).sleep();

    while(ros::ok()){
#if _STATISTICS
      time_t start = clock();
#endif

      ros::spinOnce();
#if _STATISTICS
      time_t middle = clock();
#endif
      compressor.Publish();

#if _STATISTICS
      time_t end = clock();
      double time1 = (double) (middle-start) / CLOCKS_PER_SEC * 1000.0;
      double time2 = (double) (end-middle) / CLOCKS_PER_SEC * 1000.0;
      double time = (double) (end-start) / CLOCKS_PER_SEC * 1000.0;
      std::cout << "ROS Cycle time: " << time1 << " + " << time2 << " = " << time << " ms" << std::endl;
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
