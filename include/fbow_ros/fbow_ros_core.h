#ifndef FBOW_ROS_H
#define FBOW_ROS_H

#include <stdio.h>
#include <iostream>

#include <ros/ros.h>

#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"



#ifdef USE_CONTRIB
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
using namespace cv::xfeatures2d;
#endif


//include the main fast-bag-of-words header
#include <fbow/fbow.h>

#include <stack>
#include <map> 
#include <algorithm>

using namespace cv;
using namespace std;




namespace fbow_ros {

class FBOW_ROS_Core {

public:

  FBOW_ROS_Core();
  ~FBOW_ROS_Core(){};


  ros::NodeHandle nh_;
  ros::NodeHandle nh_private;

  ros::Subscriber camera_topic_sub_;


  std::string camera_topic;

  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat img_conv;

  //vocabulary
  fbow::Vocabulary voc;
  std::string path_to_voc;
  //features
  cv::Ptr<cv::Feature2D> feature_detector;
  std::stack<cv::Mat> feature_descriptors;

  double lower_bound_score;//over which we say that we have the same place
  double upper_bound_score;//under which we say that we have a different place


  void imageCb(const sensor_msgs::ImageConstPtr& img);

  void fbow_init();

  fbow::fBow toBagOfWords(cv::Mat descriptors);
  cv::Mat computeFeatureDescriptors(cv::Mat img);
  map<double, int> compareBagOfWords(std::stack<cv::Mat> feature_descriptors, fbow::fBow fb);

  void sortAndCheckFBow(map<double, int> fbow_scores, cv::Mat descriptors);

  //function to be passed to sort(). SOrt from higher score to lower
  static bool sortByScore(const pair<double, int> &a, const pair<double, int> &b)
  {
    
    return (a.first >= b.first); 

  } 

  //print vector of pairs
  static void printVectorOfPairs(vector<pair<double, int>> vec)
  {

    // print the vector
    cout << "The map, sorted by value is: " << endl;
    for (int i = 0; i < vec.size(); i++)
    {
      cout << vec[i].first << ": " << vec[i].second << endl;
    }

  }




private:


}; // class

}; // namespace

#endif