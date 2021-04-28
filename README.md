# fbow_ros
Fast-bag-of-words ROS wrapper.

### Prerequisites

To use this package you need to have: 
* Ubuntu 16.04 LTS
* OpenCV 3.3
* ROS Kinetic

### Setup

To use this package you first have to install the main fbow package. To do so, do:
```
cd ~/catkin_ws/src
git clone https://github.com/rmsalinas/fbow
cd fbow
mkdir build && cd build
cmake ..
make
sudo make install
```

Then clone the wrapper and build it in your catkin workspace:
```
cd ~/catkin_ws/src
git clone https://github.com/NekSfyris/fbow_ros
cd ..
catkin_make
```

### Vocabulary

To create your own vocabulary, you have to create a database of images from which the features in the vocabulary will be extracted.

In my case, i created a rosbag from a D435 realsense feed (coloured) of an environment, and then converted it to images.
The database of images was kept at the following folder "~/catkin_ws/src/fbow_ros/voc_images/1".

Then, to create the actual vocabulary we first go to the build folder of the fbow package:
```
cd ~/catkin_ws/src/fbow/build/utils
```
At last, we run the scripts that do the actual work (check the scripts for more info on the parameters):
```
./fbow_create_voc_step0 orb ~/Desktop/my_voc ~/catkin_ws/src/fbow_ros/voc_images/1 
./fbow_create_voc_step1 ~/Desktop/my_voc ~/Desktop/voc.fbow -k 10 -l 6 -t 12
```

