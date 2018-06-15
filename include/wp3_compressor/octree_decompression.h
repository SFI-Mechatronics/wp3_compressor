/*
 * octree_decompression.h
 *
 *  Created on: Jan 16, 2018
 *      Author: joacim
 */

#ifndef OCTREE_DECOMPRESSION_H_
#define OCTREE_DECOMPRESSION_H_

#include <iterator>
#include <iostream>
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

typedef pcl::PointXYZI PointT_decomp;
typedef OctreePointCloudDensityContainer LeafT_decomp;
typedef OctreeContainerEmpty BranchT_decomp;
typedef Octree2BufBase<LeafT_decomp, LeafT_decomp> OctreeT_decomp;

class PointCloudDecompression : public OctreePointCloud<PointT_decomp, LeafT_decomp, BranchT_decomp, OctreeT_decomp>
{
public:
	typedef PointT_decomp PointT;
	typedef LeafT_decomp LeafT;
	typedef BranchT_decomp BranchT;
	typedef OctreeT_decomp OctreeT;

	typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloud PointCloud;
	typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloudPtr PointCloudPtr;
	typedef typename OctreePointCloud<PointT, LeafT, BranchT, OctreeT>::PointCloudConstPtr PointCloudConstPtr;

	// Boost shared pointers
	typedef boost::shared_ptr<PointCloudDecompression > Ptr;
	typedef boost::shared_ptr<const PointCloudDecompression > ConstPtr;

	typedef typename OctreeT::LeafNode LeafNode;
	typedef typename OctreeT::BranchNode BranchNode;


	/** \brief Constructor
	 *
	 */
	PointCloudDecompression ( bool showStatistics_arg = false) :
		OctreePointCloud<PointT, LeafT, BranchT, OctreeT> (0.05),
		entropy_coder_ (),
		frame_ID_ (0),
		point_count_ (0),
		i_frame_ (true),
		b_show_statistics_ (showStatistics_arg),
		pointIntensityVector_ (),
		pointIntensityVectorIterator_ (){

		frame_header_identifier_ = "<WP3-OCT-COMPRESSED>";
		//this->setResolution (octree_resolution_);

	} // End Constructor


	/** \brief Empty deconstructor. */
	virtual ~PointCloudDecompression (){

	}



	/** \brief Provide a pointer to the output data set.
	 * \param cloud_arg: the boost shared pointer to a PointCloud message
	 */
	inline void setOutputCloud (const PointCloudPtr &cloud_arg)
	{
		if (output_ != cloud_arg)
		{
			output_ = cloud_arg;
		}
	}


	/** \brief Decode point cloud from input stream
	 * \param compressed_tree_data_in_arg: binary input stream containing compressed data
	 * \param cloud_arg: reference to decoded point cloud
	 */
	void decodePointCloud (std::istream& compressed_tree_data_in_arg, PointCloudPtr &cloud_arg);


	/** \brief Get the amount of points within a leaf node voxel which is addressed by a point
	 * \param[in] point_arg: a point addressing a voxel
	 * \return amount of points that fall within leaf node voxel
	 */
	unsigned int getVoxelDensityAtPoint (const PointT& point_arg) const
	{
		unsigned int point_count = 0;

		OctreePointCloudDensityContainer* leaf = this->findLeafAtPoint (point_arg);

		if (leaf)
			point_count = leaf->getPointCounter ();

		return (point_count);
	}

private:

	/** \brief Decode leaf nodes information during deserialization
	 * \param key_arg octree key of new leaf node
	 */
	// param leaf_arg reference to new leaf node
	virtual void deserializeTreeCallback (LeafT &leaf_arg, const OctreeKey& key_arg);


	/** \brief Read frame information to output stream
	 * \param compressed_tree_data_in_arg: binary input stream
	 */
	void readFrameHeader (std::istream& compressed_tree_data_in_arg);


	/** \brief Synchronize to frame header
	 * \param compressed_tree_data_in_arg: binary input stream
	 */
	void syncToHeader (std::istream& compressed_tree_data_in_arg);


	/** \brief Entropy decoding of input binary stream and output to information vectors
	 * \param compressed_tree_data_in_arg: binary input stream
	 */
	void entropyDecoding (std::istream& compressed_tree_data_in_arg);


	/** \brief Pointer to output point cloud dataset. */
	PointCloudPtr output_;

	/** \brief Vector for storing binary tree structure */
	std::vector<char> binary_tree_data_vector_;

	/** \brief Vector for storing point intensity information  */
	std::vector<char> pointIntensityVector_;

	/** \brief Iterator on differential point information vector */
	std::vector<char>::const_iterator pointIntensityVectorIterator_;

	/** \brief Static range coder instance */
	pcl::StaticRangeCoder entropy_coder_;

	// Settings
	uint32_t frame_ID_;
	uint64_t point_count_;
	uint64_t compressed_point_data_len_;
	bool i_frame_;

	//bool activating statistics
	bool b_show_statistics_;

	//header
	const char* frame_header_identifier_;

};

}

#endif /* OCTREE_DECOMPRESSION_H_ */
