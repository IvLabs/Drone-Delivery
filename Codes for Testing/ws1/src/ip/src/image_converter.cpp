#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";
sensor_msgs::Image img;
void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
img=*msg;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::NodeHandle nh_;
     int rate = 10;
      ros::Rate r(rate);
  image_transport::ImageTransport it_(nh_); 
  image_transport::Subscriber image_sub_;
  image_sub_ = it_.subscribe("/rrbot/camera1/image_raw", 1,
      &imageCb);
  cv::namedWindow(OPENCV_WINDOW);
for(int i = 10; ros::ok() && i > 0; --i){
		ROS_INFO("width:%d height:%d ",img.width,img.height);
ros::spinOnce();
        r.sleep();
    }
  

while(1)
{
cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(img 	, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      //return;
    }

     // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
if (cv::waitKey(3) == 27) break;
ros::spinOnce();
      r.sleep();
}

  return 0;
}
