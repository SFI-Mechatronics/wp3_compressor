/*
 * defines.h
 *
 *  Created on: Jan 25, 2018
 *      Author: joacim
 */

#ifndef DEFINES_H_
#define DEFINES_H_

// ROS defines
#define _ROSRATE 60
#define _DEFAULTNODENAME "compress_pc"
#define _GLOBALFRAME "world"
#define _KINECTFRAME "_ir_optical_frame"
#define _KINECTPOINTS "/sd/points_nocolor"
#define _VELODYNEFRAME "velodyne"
#define _VELODYNEPOINTS "/velodyne_points"

#define _TOPICOUT "/wp3/"

// Compression defines
#define _STATISTICS false
//#define _OCTREERESOLUTION 0.03
#define _IFRAMERATE 20

// Filter defines
#define _MINX 0.0
#define _MINY 0.0
#define _MINZ 0.0
#define _MAXX 10.0
#define _MAXY 14.0
#define _MAXZ 4.0

//#define _MINI 2.0


#endif /* DEFINES_H_ */
