#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "FlagDetector.h"

static const std::string VIDEO_FEED = "Image window";
static const std::string THRES_FEED = "Threshold window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_color", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(VIDEO_FEED);
	///cv::namedWindow(THRES_FEED);


    std::cout<<"You should see something..."<<std::endl;
  }

  ~ImageConverter()
  {
    cv::destroyWindow(VIDEO_FEED);
	//cv::destroyWindow(THRES_FEED);

  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
	cv::Mat imgHSV;
	cv::Mat imgThres;
	
    try
    {
		// try to get own copy of image
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	//cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(3,3), 0, 0);
	//cv::cvtColor(cv_ptr->image, imgHSV, CV_BGR2HSV);
	//cv::inRange(imgHSV, cv::Scalar(170,160,60), cv::Scalar(180,256,256), imgThres);
	//cv::GaussianBlur(imgThres, imgThres, cv::Size(3,3), 0, 0);

  FlagDetector myFlagDetector;

  cv::Mat flagMat;
	
	flagMat= myFlagDetector.getBoundingBoxes(cv_ptr->image);
  //imshow("Original Image", cv_ptr->image + flagMat);
	
    // Draw an example circle on the video stream
    //if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
    //  cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    //cv::imshow(VIDEO_FEED, cv_ptr->image + flagMat);
    cv::imshow(VIDEO_FEED,  cv_ptr->image + flagMat);
	  //cv::imshow(THRES_FEED, imgThres);
    cv::waitKey(3);
    
    // Output modified video stream
	// Redirect pointer to the threshold image
	cv_ptr->image = imgThres.clone();
    image_pub_.publish(cv_ptr->toImageMsg());

	}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
