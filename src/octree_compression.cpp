/*
 * octree_compression.cpp
 *
 *  Created on: Jan 16, 2018
 *      Author: joacim
 */

#include "wp3_compressor/octree_compression.h"

namespace wp3 {

void PointCloudCompression::encodePointCloud (const PointCloudConstPtr &cloud_arg, std::ostream& compressed_tree_data_out_arg){

	// initialize octree
	this->setInputCloud (cloud_arg);

	// add point to octree
	this->addPointsFromInputCloud ();

	// make sure cloud contains points
	if (this->leaf_count_>0) {

		// if octree depth changed, we enforce I-frame encoding
		i_frame_ |= (recent_tree_depth_ != this->getTreeDepth ());// | !(iFrameCounter%10);

		// enable I-frame rate
		if (i_frame_counter_++==i_frame_rate_)
		{
			i_frame_counter_ =0;
			i_frame_ = true;
		}

		// increase frameID
		frame_ID_++;

		// initialize intensity encoding
		pointIntensityVector_.clear ();
		pointIntensityVector_.reserve (static_cast<unsigned int> (cloud_arg->points.size() ));

		// serialize octree
		if (i_frame_){
			//		//  Build tree from scratch
			//			this->deleteTree();
			//			this->setResolution (octree_resolution_);
			//			this->defineBoundingBox(minX_, minY_, minZ_, maxX_, maxY_, maxZ_);
			//			this->addPointsFromInputCloud ();
			// i-frame encoding - encode tree structure without referencing previous buffer
			this->serializeTree (binary_tree_data_vector_, false);
		}
		else
			// p-frame encoding - XOR encoded tree structure
			this->serializeTree (binary_tree_data_vector_, true);

		// write frame header information to stream
		this->writeFrameHeader (compressed_tree_data_out_arg);

		// apply entropy coding to the content of all data vectors and send data to output stream
		this->entropyEncoding (compressed_tree_data_out_arg);

		// prepare for next frame
		recent_tree_depth_ = this->getTreeDepth ();
		this->switchBuffers ();


		if (b_show_statistics_)
		{
			float bytes_per_XYZ = static_cast<float> (compressed_point_data_len_) / static_cast<float> (point_count_);
			float bytes_per_intensity = static_cast<float> (compressed_intensity_data_len_) / static_cast<float> (point_count_);

//			PCL_INFO ("*** POINTCLOUD ENCODING ***\n");
			PCL_INFO ("Frame ID: %d\n", frame_ID_);
//			if (i_frame_)
//				PCL_INFO ("Encoding Frame: Intra frame\n");
//			else
//				PCL_INFO ("Encoding Frame: Prediction frame\n");
//			PCL_INFO ("Number of encoded points (voxels): %ld\n", point_count_);
//			PCL_INFO ("XYZ compression percentage: %f%%\n", bytes_per_XYZ / (3.0f * sizeof (float)) * 100.0f);
//			PCL_INFO ("XYZ bytes per point: %f bytes\n", bytes_per_XYZ);
//			PCL_INFO ("Intensity bytes per point: %f bytes\n", bytes_per_intensity);
//
//			PCL_INFO ("Size of uncompressed point cloud: %f kBytes\n", static_cast<float> ((point_count_) * (3.0f * sizeof (float))) / 1024.0f);
//			PCL_INFO ("Size of compressed point cloud: %f kBytes\n", static_cast<float> (compressed_point_data_len_ + compressed_intensity_data_len_) / 1024.0f);
//
//			PCL_INFO ("Total encoded bytes per point: %f bytes\n", bytes_per_XYZ + bytes_per_intensity);
//			PCL_INFO ("Total compression percentage: %f%%\n", (bytes_per_XYZ + bytes_per_intensity) / (3.0f * sizeof (float)) * 100.0f);
//			PCL_INFO ("Compression ratio: %f\n\n", static_cast<float> (3.0f * sizeof (float)) / static_cast<float> (bytes_per_XYZ + bytes_per_intensity));

			logStream << point_count_ << "\t" << bytes_per_XYZ / (3.0f * sizeof (float)) * 100.0f << "\t";
			logStream << bytes_per_XYZ << "\t" << bytes_per_intensity << "\t";
			logStream << static_cast<float> ((point_count_) * (3.0f * sizeof (float))) / 1024.0f << "\t";
			logStream << static_cast<float> (compressed_point_data_len_ + compressed_intensity_data_len_) / 1024.0f << "\t";
			logStream << bytes_per_XYZ + bytes_per_intensity << "\t" << (bytes_per_XYZ + bytes_per_intensity) / (3.0f * sizeof (float)) * 100.0f << "\t";
			logStream << static_cast<float> (3.0f * sizeof (float)) / static_cast<float> (bytes_per_XYZ + bytes_per_intensity);
			logStream << std::endl;

		}

		i_frame_ = false;

	} else {
		if (b_show_statistics_)
			PCL_INFO ("Info: Dropping empty point cloud\n");
		this->deleteTree();
		this->setResolution (octree_resolution_);
		this->defineBoundingBox(minX_, minY_, minZ_, maxX_, maxY_, maxZ_);
		i_frame_counter_ = 0;
		i_frame_ = true;
	} // End if leaf_count > 0

} // End encodePointCloud()


void PointCloudCompression::writeFrameHeader (std::ostream& compressed_tree_data_out_arg)
{
	// encode header identifier
	compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (frame_header_identifier_), strlen (frame_header_identifier_));
	// encode point cloud header id
	compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&frame_ID_), sizeof (frame_ID_));
	// encode frame type (I/P-frame)
	compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&i_frame_), sizeof (i_frame_));
	if (i_frame_)
	{
		double min_x, min_y, min_z, max_x, max_y, max_z;
		double octree_resolution;

		// get current configuration
		octree_resolution = this->getResolution ();
		this->getBoundingBox (min_x, min_y, min_z, max_x, max_y, max_z);

		point_count_ = this->leaf_count_;

		// encode coding configuration
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_count_), sizeof (point_count_));
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&octree_resolution), sizeof (octree_resolution));

		// encode octree bounding box
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_x), sizeof (min_x));
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_y), sizeof (min_y));
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&min_z), sizeof (min_z));
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_x), sizeof (max_x));
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_y), sizeof (max_y));
		compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&max_z), sizeof (max_z));
	}
} // End writeFrameHeader()


void PointCloudCompression::entropyEncoding(std::ostream& compressed_tree_data_out_arg)
{
	uint64_t binary_tree_data_vector_size;
	uint64_t point_intensity_data_vector_size;

	compressed_point_data_len_ = 0;
	compressed_intensity_data_len_ = 0;

	// encode binary octree structure
	binary_tree_data_vector_size = binary_tree_data_vector_.size ();
	compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&binary_tree_data_vector_size), sizeof (binary_tree_data_vector_size));
	compressed_point_data_len_ += entropy_coder_.encodeCharVectorToStream (binary_tree_data_vector_, compressed_tree_data_out_arg);

	// encode leaf voxel intensity
	point_intensity_data_vector_size = pointIntensityVector_.size ();
	compressed_tree_data_out_arg.write (reinterpret_cast<const char*> (&point_intensity_data_vector_size), sizeof (point_intensity_data_vector_size));
	compressed_intensity_data_len_ += entropy_coder_.encodeCharVectorToStream (pointIntensityVector_, compressed_tree_data_out_arg);

	// flush output stream
	compressed_tree_data_out_arg.flush ();

} // End entropyEncoding()


void PointCloudCompression::serializeTreeCallback (LeafT &leaf_arg, const OctreeKey & key_arg)
{
	// reference to point indices vector stored within octree leaf
	const unsigned int density = leaf_arg.getPointCounter();
	leaf_arg.reset();
	unsigned char intensity;

	// get intensity
	intensity = static_cast<unsigned char> (std::max (0, std::min<int>(255, density )));
	pointIntensityVector_.push_back (intensity);

} // End serializeTreeCallback


} // End namespace wp3
