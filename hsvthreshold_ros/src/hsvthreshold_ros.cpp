#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

class HSVThreshold
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  public:
   HSVThreshold()
     : it_(nh_)
   {
       image_sub_ = it_.subscribe("/camera/image_raw", 1, &HSVThreshold::imageCb, this);
       image_pub_ = it_.advertise("/image_converter/output_video", 1);

       cv::namedWindow(OPENCV_WINDOW);
   }

   ~HSVThreshold()
   {
     cv::destroyWindow(OPENCV_WINDOW);
   }

   void imageCb(const sensor_msgs::ImageConstPtr& msg)
   {
     cv_bridge::CvImagePtr cv_ptr;
     cv::Mat HSVImage;
     cv::Mat ThreshImage;
     cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
     cv::GaussianBlur( cv_ptr->image, cv_ptr->image, cv::Size(3,3), 0, 0);
     cv::cvtColor(cv_ptr->image,HSVImage,CV_BGR2HSV);
     cv::inRange(HSVImage,Scalar(80,90,70),Scalar(110,125,200),ThreshImage);

     cv::imshow(OPENCV_WINDOW, cv_ptr->image);
     cv::waitKey(3);

     image_pub_.publish(cv_ptr->toImageMsg());
   }
};

int main(int argc, char** argv)
{
 ros::init(argc, argv, "hsv_threshold");
 HSVThreshold th;
 ros::spin();
 return 0;
}
