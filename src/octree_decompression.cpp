/*
 * octree_decompression.cpp
 *
 *  Created on: Jan 16, 2018
 *      Author: joacim
 */

#include "wp3_decompressor/octree_decompression.h"

namespace wp3 {


void PointCloudDecompression::decodePointCloud (std::istream& compressed_tree_data_in_arg, PointCloudPtr &cloud_arg){

	// synchronize to frame header
	syncToHeader(compressed_tree_data_in_arg);

	// initialize octree
	this->switchBuffers ();
	this->setOutputCloud (cloud_arg);

	// read header from input stream
	this->readFrameHeader (compressed_tree_data_in_arg);

	// decode data vectors from stream
	this->entropyDecoding (compressed_tree_data_in_arg);

	// initialize point decoding
	pointIntensityVectorIterator_ = pointIntensityVector_.begin ();

	// initialize output cloud
	output_->points.clear ();
	output_->points.reserve (static_cast<std::size_t> (point_count_));

	if (i_frame_)
		// i-frame decoding - decode tree structure without referencing previous buffer
		this->deserializeTree (binary_tree_data_vector_, false);
	else
		// p-frame decoding - decode XOR encoded tree structure
		this->deserializeTree (binary_tree_data_vector_, true);

	// assign point cloud properties
	output_->height = 1;
	output_->width = static_cast<uint32_t> (cloud_arg->points.size ());
	output_->is_dense = false;

	if (b_show_statistics_)
	{
		float bytes_per_XYZ = static_cast<float> (compressed_point_data_len_) / static_cast<float> (point_count_);

		PCL_INFO ("*** POINTCLOUD DECODING ***\n");
		PCL_INFO ("Frame ID: %d\n", frame_ID_);
//		if (i_frame_)
//			PCL_INFO ("Decoding Frame: Intra frame\n");
//		else
//			PCL_INFO ("Decoding Frame: Prediction frame\n");
//		PCL_INFO ("Number of decoded points: %ld\n", point_count_);
//		PCL_INFO ("XYZ compression percentage: %f%%\n", bytes_per_XYZ / (3.0f * sizeof (float)) * 100.0f);
//		PCL_INFO ("XYZ bytes per point: %f bytes\n", bytes_per_XYZ);
//		PCL_INFO ("Size of uncompressed point cloud: %f kBytes\n", static_cast<float> (point_count_) * (sizeof (int) + 3.0f * sizeof (float)) / 1024.0f);
//		PCL_INFO ("Size of compressed point cloud: %f kBytes\n", static_cast<float> (compressed_point_data_len_) / 1024.0f);
//		PCL_INFO ("Total bytes per point: %f bytes\n", bytes_per_XYZ);
//		PCL_INFO ("Total compression percentage: %f%%\n", (bytes_per_XYZ) / (sizeof (int) + 3.0f * sizeof (float)) * 100.0f);
//		PCL_INFO ("Compression ratio: %f\n\n", static_cast<float> (sizeof (int) + 3.0f * sizeof (float)) / static_cast<float> (bytes_per_XYZ));
	}
} // End decodePointCloud


void PointCloudDecompression::syncToHeader (std::istream& compressed_tree_data_in_arg)
{
	// sync to frame header
	unsigned int header_id_pos = 0;
	while (header_id_pos < strlen (frame_header_identifier_))
	{
		char readChar;
		compressed_tree_data_in_arg.read (static_cast<char*> (&readChar), sizeof (readChar));
		if (readChar != frame_header_identifier_[header_id_pos++])
			header_id_pos = (frame_header_identifier_[0]==readChar)?1:0;
	}
} // End syncToHeader


void PointCloudDecompression::readFrameHeader ( std::istream& compressed_tree_data_in_arg)
{
	// read header
	compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&frame_ID_), sizeof (frame_ID_));
	compressed_tree_data_in_arg.read (reinterpret_cast<char*>(&i_frame_), sizeof (i_frame_));
	if (i_frame_)
	{
		double min_x, min_y, min_z, max_x, max_y, max_z;
		double octree_resolution;

		// read coder configuration
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&point_count_), sizeof (point_count_));
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&octree_resolution), sizeof (octree_resolution));

		// read octree bounding box
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&min_x), sizeof (min_x));
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&min_y), sizeof (min_y));
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&min_z), sizeof (min_z));
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&max_x), sizeof (max_x));
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&max_y), sizeof (max_y));
		compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&max_z), sizeof (max_z));

		// reset octree and assign new bounding box & resolution
		this->deleteTree ();
		this->setResolution (octree_resolution);
		this->defineBoundingBox (min_x, min_y, min_z, max_x, max_y, max_z);

	}
} // End readFrameHeader


void PointCloudDecompression::entropyDecoding (std::istream& compressed_tree_data_in_arg)
{
	uint64_t binary_tree_data_vector_size;
	uint64_t point_intensity_data_vector_size;

	compressed_point_data_len_ = 0;

	// decode binary octree structure
	compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&binary_tree_data_vector_size), sizeof (binary_tree_data_vector_size));
	binary_tree_data_vector_.resize (static_cast<std::size_t> (binary_tree_data_vector_size));
	compressed_point_data_len_ += entropy_coder_.decodeStreamToCharVector (compressed_tree_data_in_arg, binary_tree_data_vector_);

	// decode leaf voxel intensity
	compressed_tree_data_in_arg.read (reinterpret_cast<char*> (&point_intensity_data_vector_size), sizeof (point_intensity_data_vector_size));
	pointIntensityVector_.resize (static_cast<std::size_t> (point_intensity_data_vector_size));
	compressed_point_data_len_ += entropy_coder_.decodeStreamToCharVector (compressed_tree_data_in_arg, pointIntensityVector_);

} // End entropyDecoding()


void PointCloudDecompression::deserializeTreeCallback (LeafT &leaf_arg, const OctreeKey& key_arg)
{
	PointT newPoint;

	// calculate center of lower voxel corner
	newPoint.x = static_cast<float> ((static_cast<double> (key_arg.x) + 0.5) * this->resolution_ + this->min_x_);
	newPoint.y = static_cast<float> ((static_cast<double> (key_arg.y) + 0.5) * this->resolution_ + this->min_y_);
	newPoint.z = static_cast<float> ((static_cast<double> (key_arg.z) + 0.5) * this->resolution_ + this->min_z_);

	// get point intensity
	const unsigned char& intensity = static_cast<unsigned char> (*(pointIntensityVectorIterator_++));
	newPoint.intensity = static_cast<float> (intensity);

	// add point to point cloud
	output_->points.push_back (newPoint);

} // End deserializeTreeCallback

} // End namespace wp3
