#include <fbow_ros/fbow_ros_core.h>



namespace fbow_ros{

FBOW_ROS_Core::FBOW_ROS_Core()
{
  
  cout << endl << "######################" << endl;

  //get node name and namespace
  string node_name = ros::this_node::getName();
  string node_namespace = ros::this_node::getNamespace();
  string path_to_file = node_name + node_namespace;

  nh_.param<std::string>("path_to_voc", path_to_voc, "/home/nek/catkin_ws/src/fbow_ros/vocabularies/orb_mur.fbow");
  nh_.param<double>("lower_bound_score", lower_bound_score, 0.07);
  nh_.param<double>("upper_bound_score", upper_bound_score, 0.005);
  nh_.param<std::string>("camera_topic", camera_topic, "/t265/undistorted/left/image_raw");


  //Subscribe to camera feed
  camera_topic_sub_ = nh_.subscribe(camera_topic, 10, &FBOW_ROS_Core::imageCb, this);


  fbow_init();

}



//------------------------ Sensor topic callbacks ------------------------//

void FBOW_ROS_Core::imageCb(const sensor_msgs::ImageConstPtr& img)
{

  try
  {
    if (sensor_msgs::image_encodings::isColor(img->encoding))
    {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
      //std::cout << "Got BGR8!" << std::endl;

      //Convert the image to grayscale. Parameters: (input, output, color conversion)
      cv::cvtColor(cv_ptr->image, img_conv, CV_BGR2GRAY);
      cv_ptr->image = img_conv;
      cv_ptr->encoding = "mono8";//need to fill the encoding
      //Check in case something went wrong with the conversion
      assert(cv_ptr->image.type() == CV_8U);
      assert(cv_ptr->image.channels() == 1);
    }
    else
    {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
      //std::cout << "Got MONO8!" << std::endl;
      img_conv = cv_ptr->image;
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

}


//--------------------------------------------------------------------------//


void FBOW_ROS_Core::fbow_init()
{

  ros::Rate rate(30);//rate in Hz
  
  //read the vocabulary
  voc.readFromFile(path_to_voc);
  
  if(voc.isValid())
  {

    //get descriptor name and type
    string descr_name = voc.getDescName();
    uint32_t descr_type = voc.getDescType();
    cout<< "Vocabulary uses: " << descr_name << ",  of type: " << descr_type << endl;
    if(descr_name == "orb")
    {
      feature_detector = cv::ORB::create(3000);
      cout<< "Feature type supported!" << endl;
    }
    #ifdef OPENCV_VERSION_3
    else if (descr_name == "akaze")
      fdetector = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 1e-4);
    #endif
    #ifdef USE_CONTRIB
    else if (descr_name == "surf")
      fdetector = cv::xfeatures2d::SURF::create(15, 4, 2);
    #endif
    else 
      throw std::runtime_error("Invalid descriptor");
    assert(!descr_name.empty());
        


    int frame_id = 0;
    int max_kf_rate = 150;

    while(ros::ok())
    {

      ros::spinOnce();

      //use every ith frame
      frame_id++;
      if(frame_id%max_kf_rate==0)
      {

        frame_id=0;
        
        //compute descriptors
        cv::Mat descriptors = computeFeatureDescriptors(img_conv);

        //to bag-of-words
        fbow::fBow fb;
        fb = toBagOfWords(descriptors);

        //check if we have been to this place again by comparing bag-of-words
        map<double, int> fbow_scores;
        fbow_scores = compareBagOfWords(feature_descriptors, fb);


        //time to decidfe if you are going to consider this KF as a new place
        sortAndCheckFBow(fbow_scores, descriptors);

      }
      // Sleep until we need to publish a new measurement.
      rate.sleep();
    }

  }

}



//compute and return the feature descriptos in image
cv::Mat FBOW_ROS_Core::computeFeatureDescriptors(cv::Mat img)
{

  std::vector<KeyPoint> keypoints;

  cv::Mat descriptors;

  feature_detector->detectAndCompute(img, cv::Mat(), keypoints, descriptors);
  return descriptors;

}

//convert from feature descriptors to bag-of-words representation
fbow::fBow FBOW_ROS_Core::toBagOfWords(cv::Mat descriptors)
{

  fbow::fBow fb;
  fb = voc.transform(descriptors);

  return fb;

}

//compare current bag-of-words with all registered places
map<double, int> FBOW_ROS_Core::compareBagOfWords(std::stack<cv::Mat> fdescriptors, fbow::fBow fb)
{

  map<double, int> fbow_scores;

  if(!fdescriptors.empty())
  {

    std::cout << "FBow scores: ";

    int i = fdescriptors.size()-1;

    //go through all 'places' we have been to
    while(!fdescriptors.empty()) 
    {

      fbow::fBow fb_temp;

      //to bag-of-words
      fb_temp = voc.transform(fdescriptors.top());

      //when compared to itself it gives approximately 1. The less similar the places are, the smaller the output score.
      double score_temp = fb.score(fb, fb_temp);

      fbow_scores.insert(pair<double, int>(score_temp, i));
      std::cout << score_temp << " ";

      fdescriptors.pop();

      i--;

    } 

    cout << endl;

  }

  return fbow_scores;

}



//sort FBow scores you have computed and then check if you are going to add the KF info as a new place
void FBOW_ROS_Core::sortAndCheckFBow(map<double, int> fbow_scores, cv::Mat descriptors)
{

  if(fbow_scores.empty())
  {

    //put first KF's descriptors in the stack
    feature_descriptors.push(descriptors);
    cout << "ADDED A NEW PLACE!" << endl;

  }
  else
  {

    // create a empty vector of pairs
    vector<pair<double, int>> vec;

    // copy key-value pairs from the map to the vector
    map<double, int> :: iterator it;
    for (it=fbow_scores.begin(); it!=fbow_scores.end(); it++) 
    {
      //cout << it->first << " " << it->second << endl;
      vec.push_back(make_pair(it->first, it->second));
    }

    // sort the vector by increasing order of its pair's second value
    sort(vec.begin(), vec.end(), sortByScore); 

    // print the vector
    //printVectorOfPairs(vec);

    //always compare with the "worst" case, which is the first place in the sorted fbow scores
    if(vec[0].first>=lower_bound_score)//were we at the same place before?
    {

      cout << "YOU HAVE BEEN AT THIS PLACE BEFORE!" << endl;

    }
    else if(vec[0].first<=upper_bound_score)//is this a new place?
    {

      //put this KF's descriptors in the stack
      feature_descriptors.push(descriptors);
      cout << "ADDED A NEW PLACE!" << endl;

    }

  }
  

}









};// namespace







//--------------------//
//------  MAIN  ------//
//--------------------//
int main(int argc, char** argv)
{

  ros::init(argc, argv, "fbow_ros_core_node");
  ros::NodeHandle nh_private("~");

  fbow_ros::FBOW_ROS_Core fbow_obj;
  ros::spin();
  return(0);

}