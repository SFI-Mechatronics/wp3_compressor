# wp3_compressor
ROS node for compressing and transmitting 3D sensor data on an NVIDIA Jetson TX2. The work is based on the Point Cloud Library's octree compressor.

**Keywords:** ROS, RGB-D, compression

### License
The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Joacim Dybedal<br />
Affiliation: [SFI Offshore Mechatronics](https://sfi.mechatronics.no/), [University of Agder](https://www.uia.no/en)<br />
Maintainer: Joacim Dybedal, joacim.dybedald@uia.no**

The wp3_compressor package has been tested under ROS Kinetic Kame and Ubuntu 16.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Publications

If you use this work in an academic context, please cite the following publication:

* J. Dybedal, A. Aalerud, and G. Hovland, “**Embedded Processing and Compression of 3D Sensor Data for Large Scale Industrial Environments**,”. Sensors, vol. 19, no. 3, p. 636, Feb. 2019.

        @article{Dybedal2019,
                author = {Dybedal, Joacim and Aalerud, Atle and Hovland, Geir},
                title = {{Embedded Processing and Compression of 3D Sensor Data for Large Scale Industrial Environments}},
                doi = {10.3390/s19030636},
                journal = {Sensors},
                month = {feb},
                number = {3},
                pages = {636},
                publisher = {Multidisciplinary Digital Publishing Institute},
                volume = {19},
                year = {2019}
        }

## Installation
### Building from Source
#### Dependencies
- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Point Cloud Library](http://www.pointclouds.org)

#### Building
To build from source, clone the latest version from this repository into your catkin workspace and compile the package using
````bash
cd catkin_workspace/src
git clone https://github.com/SFI-Mechatronics/wp3_compressor.git
cd ../
catkin_make
````

## Usage
The compressor can be started by using the **.launch** files found in the **launch/** folder. 
These contain parameters such as topic names, sensor name and resolution.

## Bugs & Feature Requests
Please report bugs and request features using the [Issue Tracker](https://github.com/SFI-Mechatronics/wp3_compressor/issues).
