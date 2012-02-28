#include <ros/ros.h>
#include <ros/console.h>
#include "libhueblob/object.hh"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/image_encodings.h>
#include <hueblob/RoiStamped.h>


#include "cv.h"
#include "highgui.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

using namespace std;
namespace enc = sensor_msgs::image_encodings;
class Tracker2D
{
public:
  Tracker2D();
  virtual ~Tracker2D(){};

private:
  void imageCallback(const sensor_msgs::ImageConstPtr& image);

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter sub_;
  ros::Publisher roi_pub_;
  image_transport::Publisher tracked_image_pub_, ;
  image_transport::Publisher hsv_image_pub_, bgr_image_pub_;
  image_transport::Publisher gray_image_pub_, mono_image_pub_;
  string image_, model_path_, name_;
  Object object_;
  cv_bridge::CvImagePtr cv_ptr_, hsv_ptr_, bgr_ptr_, gray_ptr_, mono_ptr_;
};


Tracker2D::Tracker2D()
  : nh_("track_2d"),
    it_(nh_),
    sub_(),
    roi_pub_(),
    tracked_image_pub_(),
    image_(),
    model_path_(),
    name_(),
    object_(),
    cv_ptr_(),
    hsv_ptr_(new cv_bridge::CvImage),
    bgr_ptr_(new cv_bridge::CvImage),
    gray_ptr_(new cv_bridge::CvImage),
    mono_ptr_(new cv_bridge::CvImage)
{

  ros::param::param<string>("~image", image_, "left/image_rect_color");
  ros::param::param<string>("~name", name_, "test");
  ros::param::param<string>("~model", model_path_, "/home/nddang/src/ros/hueblob/data/models/ball-rose-5.png");

  const::string image_topic = ros::names::resolve(image_);
  cv::Mat model = cv::imread(model_path_.c_str());
  object_.addView(model);

  const::string roi_topic = ros::names::resolve("blobs/" + name_ + "/blob2d");
  const::string tracked_image_topic = ros::names::resolve("blobs/" + name_ + "/tracked_image");
  const::string hsv_image_topic = ros::names::resolve("blobs/" + name_ + "/hsv_image");
  const::string bgr_image_topic = ros::names::resolve("blobs/" + name_ + "/bgr_image");
  const::string gray_image_topic = ros::names::resolve("blobs/" + name_ + "/gray_image");
  const::string mono_image_topic = ros::names::resolve("blobs/" + name_ + "/mono_image");

  roi_pub_ = nh_.advertise<hueblob::RoiStamped>(roi_topic, 5);
  tracked_image_pub_ = it_.advertise(tracked_image_topic, 1);
  hsv_image_pub_ = it_.advertise(hsv_image_topic, 1);
  bgr_image_pub_ = it_.advertise(bgr_image_topic, 1);
  gray_image_pub_ = it_.advertise(gray_image_topic, 1);
  mono_image_pub_ = it_.advertise(mono_image_topic, 1);

  sub_.subscribe(it_, image_topic, 5);
  sub_.registerCallback(boost::bind(&Tracker2D::imageCallback,
                                    this, _1));

  ROS_INFO_STREAM("Listening to " << image_topic);

}

void Tracker2D::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
    {
      cv_ptr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  boost::optional<cv::RotatedRect> rrect = object_.track(cv_ptr_->image);
  cv::Rect rect(-1, -1, -1, -1);
  if (!rrect)
    {
      ROS_WARN_THROTTLE(20, "failed to track object");
      return;
    }
  else
    rect = rrect->boundingRect();
  // ROS_INFO_STREAM(rect.x << " "<< rect.y << " "<< rect.width << " " <<rect.height);
  if (0 <= rect.x && 0 < rect.width &&  cv_ptr_->image.cols > rect.x &&
      rect.x + rect.width <= cv_ptr_->image.cols &&
      0 <= rect.y && 0 <= rect.height &&  cv_ptr_->image.rows > rect.y &&
      rect.y + rect.height <= cv_ptr_->image.rows)
    {
      hueblob::RoiStamped r;
      r.header = msg->header;
      r.roi.x_offset = rect.x;
      r.roi.y_offset = rect.y;
      r.roi.width = rect.width;
      r.roi.height = rect.height;
      r.roi.do_rectify = true;

      hsv_ptr_->header = cv_ptr_->header;
      hsv_ptr_->encoding = cv_ptr_->encoding;
      hsv_ptr_->image = object_.imgHSV_(rect);

      bgr_ptr_->header = cv_ptr_->header;
      bgr_ptr_->encoding = cv_ptr_->encoding;
      bgr_ptr_->image = cv_ptr_->image(rect);

      gray_ptr_->header = cv_ptr_->header;
      gray_ptr_->encoding = "mono8";

      mono_ptr_->header = cv_ptr_->header;
      mono_ptr_->encoding = "mono8";

      // cv::Mat mask(cv_ptr_->image.rows,
      //              cv_ptr_->image.cols, CV_8U);

      // hsv_ptr_->image.create(cv_ptr_->image.rows,
      //                       cv_ptr_->image.cols, CV_8UC3);

      // cv::inRange(object_.imgHSV_, object_.lower_hsv_,
      //             object_.upper_hsv_, mask);

      // object_.imgHSV_.copyTo(hsv_ptr_->image, mask);

      // ROS_INFO_STREAM(object_.lower_hsv_[0] << " " << object_.lower_hsv_[1] << " " << object_.lower_hsv_[2] << " "
      //                 << object_.upper_hsv_[0] << " " << object_.upper_hsv_[1] << " " << object_.upper_hsv_[2] << " "
      //                 << object_.peak_color_[0] << " " <<
      //                 object_.peak_color_[1] << " " <<
      //                 object_.peak_color_[2] << " ");

      // Find contours

      static const int THRESH = 200;
      cv::Mat rc, gr, bc;
      std::vector<cv::Mat> channels;
      //gray_ptr_->image.create(bgr_ptr_->image.cols, bgr_ptr_->image.cols, CV_8UC1);
      //cv::cvtColor( bgr_ptr_->image, gray_ptr_->image, CV_BGR2GRAY );
      cv::split(bgr_ptr_->image, channels);
      gray_ptr_->image = channels[2];
      cv::threshold(gray_ptr_->image, mono_ptr_->image,
                    THRESH, 255, CV_THRESH_BINARY );

      // std::vector< std::vector<cv::Point2i> > contours;
      // std::vector<cv::Vec4i> hierarchy;

      // cv::findContours(gray_ptr_->image, contours,
      //                  CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE );
      // cv::drawContours(gray_ptr_->image,
      //                  contours,
      //                  -1,
      //                  cv::Scalar(255,0,0)
      //                  );


      hsv_image_pub_.publish(hsv_ptr_->toImageMsg());
      bgr_image_pub_.publish(bgr_ptr_->toImageMsg());
      gray_image_pub_.publish(gray_ptr_->toImageMsg());
      mono_image_pub_.publish(mono_ptr_->toImageMsg());

      roi_pub_.publish(r);
    }



  if ( tracked_image_pub_.getNumSubscribers() != 0)
    {
      const cv::Scalar color = CV_RGB(255,0,0);
      if (rrect)
        {
          cv::Point p1(rect.x, rect.y);
          cv::Point p2(rect.x + rect.width, rect.y + rect.height);
          cv::Point pc(rect.x, rect.y + std::max(16, rect.height+8));

          ROS_DEBUG_STREAM("Drawing rect " << rect.x << " " << " " << rect.y
                           << " " << rect.width << " " << rect.height);

          cv::rectangle(cv_ptr_->image, p1, p2, color, 1);
          std::stringstream ss (std::stringstream::in
                                | std::stringstream::out);
          cv::putText(cv_ptr_->image, name_, p1, CV_FONT_HERSHEY_SIMPLEX,
                      0.5, color);
        }
      else
        {
          cv::Point p1(10,10);
          cv::putText(cv_ptr_->image, "Lost", p1, CV_FONT_HERSHEY_SIMPLEX,
                      0.5, color);
        }
      tracked_image_pub_.publish(cv_ptr_->toImageMsg());
    }


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracker_2d");
  Tracker2D tracker;
  ros::spin();
}
