/*
 * octree_compression.h
 *
 *  Created on: Jan 16, 2018
 *      Author: joacim
 */

#ifndef OCTREE_COMPRESSION_H_
#define OCTREE_COMPRESSION_H_

#include <iterator>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <string.h>

#include <pcl/common/common.h>
#include <pcl/common/io.h>

#include <pcl/octree/octree2buf_base.h>
#include <pcl/octree/impl/octree2buf_base.hpp>

#include <pcl/octree/octree_pointcloud_density.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/impl/octree_pointcloud.hpp>

#include <pcl/compression/entropy_range_coder.h>


using namespace pcl::octree;

namespace wp3 {

typedef pcl::PointXYZI PointType;
typedef OctreePointCloudDensityContainer LeafT;
typedef OctreeContainerEmpty BranchT;
typedef Octree2BufBase<LeafT, BranchT> OctreeT;

class PointCloudCompression : public OctreePointCloud<PointType, LeafT, BranchT, OctreeT>
{
public:

  typedef typename OctreePointCloud<PointType, LeafT, BranchT, OctreeT>::PointCloud PointCloud;
  typedef typename OctreePointCloud<PointType, LeafT, BranchT, OctreeT>::PointCloudPtr PointCloudPtr;
  typedef typename OctreePointCloud<PointType, LeafT, BranchT, OctreeT>::PointCloudConstPtr PointCloudConstPtr;

  // Boost shared pointers
  typedef boost::shared_ptr<PointCloudCompression > Ptr;
  typedef boost::shared_ptr<const PointCloudCompression > ConstPtr;

  typedef typename OctreeT::LeafNode LeafNode;
  typedef typename OctreeT::BranchNode BranchNode;

  /** \brief Constructor
   *
   */
  PointCloudCompression (
      bool showStatistics_arg = false,
      const double octreeResolution_arg = 0.01,
      const double minX_arg = 0, const double minY_arg = 0, const double minZ_arg = 0,
      const double maxX_arg = 10, const double maxY_arg = 10, const double maxZ_arg = 10,
      const unsigned int iFrameRate_arg = 30 ):
    OctreePointCloud<PointType, LeafT, BranchT, OctreeT> (octreeResolution_arg),
    entropy_coder (),
    i_frame_rate (iFrameRate_arg),
    i_frame_counter (0),
    frame_ID (0),
    point_count (0),
    octree_resolution(octreeResolution_arg),
    i_frame (true),
    b_show_statistics (showStatistics_arg),
    pointIntensityVector (),
    recent_tree_depth(0),
    minX(minX_arg), minY(minY_arg), minZ(minZ_arg),
    maxX(maxX_arg), maxY(maxY_arg), maxZ(maxZ_arg),
    logFile("/home/nvidia/encoderlog.txt"){

    if(b_show_statistics){
      logStream.open(logFile.c_str());
      logStream << "NumPoints\tXYZpc\tXYZbpp\tINTbpp\tOsize\tCsize\tBPP\tCPC\tRATIO" << std::endl;
    }

    frame_header_identifier = "<WP3-OCT-COMPRESSED>";

    this->setResolution (octree_resolution);
    this->defineBoundingBox(minX, minY, minZ, maxX, maxY, maxZ);

  } // End Constructor


  /** \brief Deconstructor. */
  virtual ~PointCloudCompression (){
    if(b_show_statistics)
      logStream.close();
  }


  /** \brief Provide a pointer to the output data set.
   * \param cloud_arg: the boost shared pointer to a PointCloud message
   */
  inline void setOutputCloud (const PointCloudPtr &cloud_arg)
  {
    if (output != cloud_arg)
    {
      output = cloud_arg;
    }
  }


  /** \brief Encode point cloud to output stream
   * \param cloud_arg:  point cloud to be compressed
   * \param compressed_tree_data_out_arg:  binary output stream containing compressed data
   */
  void encodePointCloud (const PointCloudConstPtr &cloud_arg, std::ostream& compressed_tree_data_out_arg);


  /** \brief Get the amount of points within a leaf node voxel which is addressed by a point
   * \param[in] point_arg: a point addressing a voxel
   * \return amount of points that fall within leaf node voxel
   */
  unsigned int getVoxelDensityAtPoint (const PointType& point_arg) const
  {
    unsigned int point_count = 0;

    OctreePointCloudDensityContainer* leaf = this->findLeafAtPoint (point_arg);

    if (leaf)
      point_count = leaf->getPointCounter ();

    return (point_count);
  }

private:


  virtual void serializeTreeCallback (LeafT &leaf_arg, const OctreeKey& key_arg);

  /** \brief Write frame information to output stream
   * \param compressed_tree_data_out_arg: binary output stream
   */
  void writeFrameHeader (std::ostream& compressed_tree_data_out_arg);

  /** \brief Apply entropy encoding to encoded information and output to binary stream
   * \param compressed_tree_data_out_arg: binary output stream
   */
  void entropyEncoding(std::ostream& compressed_tree_data_out_arg);

  /** \brief Pointer to output point cloud dataset. */
  PointCloudPtr output;

  /** \brief Vector for storing binary tree structure */
  std::vector<char> binary_tree_data_vector;

  /** \brief Vector for storing point intensity information  */
  std::vector<char> pointIntensityVector;

  /** \brief Static range coder instance */
  pcl::StaticRangeCoder entropy_coder;

  // Settings
  uint32_t i_frame_rate;
  uint32_t i_frame_counter;
  uint32_t frame_ID;
  uint64_t point_count;
  uint64_t compressed_point_data_len;
  uint64_t compressed_intensity_data_len;
  bool i_frame;
  const double octree_resolution;

  unsigned int recent_tree_depth;

  const double minX;
  const double minY;
  const double minZ;
  const double maxX;
  const double maxY;
  const double maxZ;

  //bool activating statistics
  bool b_show_statistics;

  //header
  const char* frame_header_identifier;

  // Logging
  std::string logFile;
  std::ofstream logStream;

};

}

#endif /* OCTREE_COMPRESSION_H_ */
