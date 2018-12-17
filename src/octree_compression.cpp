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
    i_frame |= (recent_tree_depth != this->getTreeDepth ());// | !(iFrameCounter%10);

    // enable I-frame rate
    if (i_frame_counter++==i_frame_rate)
    {
      i_frame_counter =0;
      i_frame = true;
    }

    // increase frameID
    frame_ID++;

    // initialize intensity encoding
    pointIntensityVector.clear ();
    pointIntensityVector.reserve (static_cast<unsigned int> (cloud_arg->points.size() ));

    // serialize octree
    if (i_frame){
      //		//  Build tree from scratch
      //			this->deleteTree();
      //			this->setResolution (octree_resolution_);
      //			this->defineBoundingBox(minX_, minY_, minZ_, maxX_, maxY_, maxZ_);
      //			this->addPointsFromInputCloud ();
      // i-frame encoding - encode tree structure without referencing previous buffer
      this->serializeTree (binary_tree_data_vector, false);
    }
    else
      // p-frame encoding - XOR encoded tree structure
      this->serializeTree (binary_tree_data_vector, true);

    // write frame header information to stream
    this->writeFrameHeader (compressed_tree_data_out_arg);

    // apply entropy coding to the content of all data vectors and send data to output stream
    this->entropyEncoding (compressed_tree_data_out_arg);

    // prepare for next frame
    recent_tree_depth = this->getTreeDepth ();
    this->switchBuffers ();


    if (b_show_statistics)
    {
      float bytes_per_XYZ = static_cast<float> (compressed_point_data_len) / static_cast<float> (point_count);
      float bytes_per_intensity = static_cast<float> (compressed_intensity_data_len) / static_cast<float> (point_count);

      //			PCL_INFO ("*** POINTCLOUD ENCODING ***\n");
      PCL_INFO ("Frame ID: %d\n", frame_ID);
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

      logStream << point_count << "\t" << bytes_per_XYZ / (4.0f * sizeof (float)) * 100.0f << "\t";
      logStream << bytes_per_XYZ << "\t" << bytes_per_intensity << "\t";
      logStream << static_cast<float> ((point_count) * (4.0f * sizeof (float))) / 1024.0f << "\t";
      logStream << static_cast<float> (compressed_point_data_len + compressed_intensity_data_len) / 1024.0f << "\t";
      logStream << bytes_per_XYZ + bytes_per_intensity << "\t" << (bytes_per_XYZ + bytes_per_intensity) / (4.0f * sizeof (float)) * 100.0f << "\t";
      logStream << static_cast<float> (4.0f * sizeof (float)) / static_cast<float> (bytes_per_XYZ + bytes_per_intensity);
      logStream << std::endl;

    }

    i_frame = false;

  } else {
    PCL_INFO ("Info: Dropping empty point cloud\n");
    this->deleteTree();
    this->setResolution (octree_resolution);
    this->defineBoundingBox(minX, minY, minZ, maxX, maxY, maxZ);
    i_frame_counter = 0;
    i_frame = true;
  } // End if leaf_count > 0

} // End encodePointCloud()

void PointCloudCompression::writeFrameHeader (std::ostream & compressed_tree_data_out_arg)
{
  // encode header identifier
  compressed_tree_data_out_arg.write (reinterpret_cast<const char *> (frame_header_identifier), strlen (frame_header_identifier));
  // encode point cloud header id
  compressed_tree_data_out_arg.write (reinterpret_cast<const char *> (&frame_ID), sizeof (frame_ID));
  // encode frame type (I/P-frame)
  compressed_tree_data_out_arg.write (reinterpret_cast<const char *> (&i_frame), sizeof (i_frame));
  if (i_frame)
  {
    double min_x, min_y, min_z, max_x, max_y, max_z;
    double octree_resolution;

    // get current configuration
    octree_resolution = this->getResolution ();
    this->getBoundingBox (min_x, min_y, min_z, max_x, max_y, max_z);

    point_count = this->leaf_count_;

    // encode coding configuration
    compressed_tree_data_out_arg.write (reinterpret_cast<const char *> (&point_count), sizeof (point_count));
    compressed_tree_data_out_arg.write (reinterpret_cast<const char *> (&octree_resolution), sizeof (octree_resolution));

    // encode octree bounding box
    compressed_tree_data_out_arg.write (reinterpret_cast<const char *> (&min_x), sizeof (min_x));
    compressed_tree_data_out_arg.write (reinterpret_cast<const char *> (&min_y), sizeof (min_y));
    compressed_tree_data_out_arg.write (reinterpret_cast<const char *> (&min_z), sizeof (min_z));
    compressed_tree_data_out_arg.write (reinterpret_cast<const char *> (&max_x), sizeof (max_x));
    compressed_tree_data_out_arg.write (reinterpret_cast<const char *> (&max_y), sizeof (max_y));
    compressed_tree_data_out_arg.write (reinterpret_cast<const char *> (&max_z), sizeof (max_z));
  }
} // End writeFrameHeader()

void PointCloudCompression::entropyEncoding(std::ostream & compressed_tree_data_out_arg)
{
  uint64_t binary_tree_data_vector_size;
  uint64_t point_intensity_data_vector_size;

  compressed_point_data_len = 0;
  compressed_intensity_data_len = 0;

  // encode binary octree structure
  binary_tree_data_vector_size = binary_tree_data_vector.size ();
  compressed_tree_data_out_arg.write (reinterpret_cast<const char *> (&binary_tree_data_vector_size), sizeof (binary_tree_data_vector_size));
  compressed_point_data_len += entropy_coder.encodeCharVectorToStream (binary_tree_data_vector, compressed_tree_data_out_arg);

  // encode leaf voxel intensity
  point_intensity_data_vector_size = pointIntensityVector.size ();
  compressed_tree_data_out_arg.write (reinterpret_cast<const char *> (&point_intensity_data_vector_size), sizeof (point_intensity_data_vector_size));
  compressed_intensity_data_len += entropy_coder.encodeCharVectorToStream (pointIntensityVector, compressed_tree_data_out_arg);

  // flush output stream
  compressed_tree_data_out_arg.flush ();

} // End entropyEncoding()

void PointCloudCompression::serializeTreeCallback (LeafT & leaf_arg, const OctreeKey & key_arg)
{
  const char intens = static_cast<unsigned char>(
        std::max<int>(0, std::min(leaf_arg.getIntensity(),255.0f)));

  leaf_arg.reset();
  pointIntensityVector.push_back(intens);
} // End serializeTreeCallback

} // End namespace wp3
