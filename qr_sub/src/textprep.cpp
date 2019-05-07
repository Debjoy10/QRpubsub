#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <iomanip>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/Point32.h"
#include "barcode/qrdata.h"
#include "barcode.h"
using namespace std;
using namespace cv;


cv::Mat frame;
cv_bridge::CvImage img_bridge;
sensor_msgs::Image img_msg;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void QRdataCallback(const barcode::qrdata::ConstPtr& msg)
{
  ROS_INFO("Data: [%s]", msg->data.c_str());
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "textprep");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("/code/image", 1, imageCallback);
	ros::Subscriber data_sub = nh.subscribe<barcode::qrdata>("/code/data",1,QRdataCallback);
	ros::spin();
	return 0;
}
		
	
	
