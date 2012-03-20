#include <ros/ros.h>
#include <ros/console.h>
#include "libhueblob/object.hh"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/image_encodings.h>
#include <hueblob/RoiStamped.h>
#include <hueblob/RotatedRectStamped.h>


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
  void hintCallback(const sensor_msgs::RegionOfInterestConstPtr& roi);

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter sub_;
  ros::Subscriber hint_sub_;
  ros::Publisher roi_pub_, rrect_pub_;
  image_transport::Publisher tracked_image_pub_, ;
  image_transport::Publisher hsv_image_pub_, bgr_image_pub_, mono_image_pub_;
  string image_, model_path_, name_;
  Object object_;
  cv_bridge::CvImagePtr cv_ptr_, hsv_ptr_, bgr_ptr_, mono_ptr_;
};


Tracker2D::Tracker2D()
  : nh_("track_2d"),
    it_(nh_),
    sub_(),
    roi_pub_(),
    rrect_pub_(),
    tracked_image_pub_(),
    image_(),
    model_path_(),
    name_(),
    object_(),
    cv_ptr_(),
    hsv_ptr_(new cv_bridge::CvImage),
    bgr_ptr_(new cv_bridge::CvImage),
    mono_ptr_(new cv_bridge::CvImage)
{

  ros::param::param<string>("~image", image_, "left/image_rect_color");
  ros::param::param<string>("~name", name_, "test");
  ros::param::param<string>("~model", model_path_,
                            "/home/nddang/src/ros/hueblob/data/models/ball-rose-5.png");

  const::string image_topic = ros::names::resolve(image_);
  cv::Mat model = cv::imread(model_path_.c_str());
  object_.addView(model);

  const::string hint_topic          = ros::names::resolve("blobs/" + name_ + "/hint");
  const::string roi_topic           = ros::names::resolve("blobs/" + name_ + "/roi");
  const::string rrect_topic         = ros::names::resolve("blobs/" + name_ + "/rrect");
  const::string tracked_image_topic = ros::names::resolve("blobs/" + name_ + "/tracked_image");
  const::string hsv_image_topic     = ros::names::resolve("blobs/" + name_ + "/hsv_image");
  const::string bgr_image_topic     = ros::names::resolve("blobs/" + name_ + "/bgr_image");
  const::string mono_image_topic    = ros::names::resolve("blobs/" + name_ + "/mono_image");

  roi_pub_ = nh_.advertise<hueblob::RoiStamped>(roi_topic, 5);
  rrect_pub_ = nh_.advertise<hueblob::RotatedRectStamped>(rrect_topic, 5);
  tracked_image_pub_ = it_.advertise(tracked_image_topic, 1);
  hsv_image_pub_ = it_.advertise(hsv_image_topic, 1);
  bgr_image_pub_ = it_.advertise(bgr_image_topic, 1);
  mono_image_pub_ = it_.advertise(mono_image_topic, 1);

  sub_.subscribe(it_, image_topic, 5);
  sub_.registerCallback(boost::bind(&Tracker2D::imageCallback,
                                    this, _1));
  hint_sub_ = nh_.subscribe(hint_topic, 5,
                            &Tracker2D::hintCallback, this);



  ROS_INFO_STREAM("Listening to " << image_topic << ", " << hint_topic);

}

void Tracker2D::hintCallback(const sensor_msgs::RegionOfInterestConstPtr& roi)
{
  cv::Rect rect(roi->x_offset, roi->y_offset, roi->width, roi->height);
  object_.setSearchWindow(rect);
  // ROS_INFO_STREAM("Set rect " << rect.x
  //                 << " " << rect.y
  //                 << " " << rect.width
  //                 << " " << rect.height);
  return;
}

namespace
{
  bool rotated_rect(cv::Mat im, const cv::RotatedRect & rrect, cv::Scalar color)
  {
    // ROS_INFO_STREAM("INVALID RRECT " << rrect.center.x << " " << rrect.center.y << " "
    //                 << rrect.size.width << "  " << rrect.size.height << " "
    //                 << rrect.angle);
    CvPoint2D32f box_vtx[4];
    cvBoxPoints(rrect, box_vtx);

    cv::Point pt0, pt;
    pt0.x = cvRound(box_vtx[3].x);
    pt0.y = cvRound(box_vtx[3].y);

    if( (pt0.x <= 0) || (pt0.x >= im.cols)
        || (pt0.y <= 0) ||( pt0.y >= im.rows))
      return false;

    for(int i = 0; i < 4; i++ )
      {
        pt.x = cvRound(box_vtx[i].x);
        pt.y = cvRound(box_vtx[i].y);
        if ( (pt.x <= 0) || (pt.x >= im.cols)
             || (pt.y <= 0) || (pt.y >= im.rows) )
          return false;
        cv::line(im, pt0, pt, color, 1, CV_AA, 0);
        pt0 = pt;
      }
    return true;
  }
}

void Tracker2D::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  static const cv::Scalar color =  CV_RGB(255,0,0);
  static const cv::Scalar color2 = CV_RGB(0,0,255);
  static const cv::Scalar color3 = CV_RGB(0,255,0);
  static const cv::Scalar white  = CV_RGB(255,255,255);

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

  if ( tracked_image_pub_.getNumSubscribers() != 0)
    {
      if (rrect)
        {
          cv::Point p1(rect.x, rect.y);
          cv::Point p2(rect.x + rect.width, rect.y + rect.height);
          cv::Point pc(rect.x, rect.y + std::max(16, rect.height+8));

          ROS_DEBUG_STREAM("Drawing rect " << rect.x << " "
                           << " " << rect.y
                           << " " << rect.width <<
                           " " << rect.height);
          cv::putText(cv_ptr_->image, name_, p1,
                      CV_FONT_HERSHEY_SIMPLEX,
                      0.5, color);
          if (rotated_rect(cv_ptr_->image, *rrect, color))
            {
              cv::ellipse(cv_ptr_->image, *rrect, color2);
              cv::rectangle(cv_ptr_->image, p1, p2, color3);
            }
        }
      else
        {
          cv::Point p1(10,10);
          cv::putText(cv_ptr_->image, "Lost",
                      p1, CV_FONT_HERSHEY_SIMPLEX,
                      0.5, color);
        }
    }

  if (0 <= rect.x && 0 <= rect.width &&
      rect.x + rect.width < cv_ptr_->image.cols &&
      0 <= rect.y && 0 <= rect.height &&
      rect.y + rect.height < cv_ptr_->image.rows)
    {
      hueblob::RoiStamped r;
      r.header = msg->header;
      r.roi.x_offset = rect.x;
      r.roi.y_offset = rect.y;
      r.roi.width = rect.width;
      r.roi.height = rect.height;
      r.roi.do_rectify = true;

      hueblob::RotatedRectStamped rrect_msg;
      rrect_msg.header = msg->header;
      rrect_msg.rrect.x = rrect->center.x;
      rrect_msg.rrect.y = rrect->center.y;
      rrect_msg.rrect.width = rrect->size.width;
      rrect_msg.rrect.height = rrect->size.height;
      rrect_msg.rrect.angle = rrect->angle;

      hsv_ptr_->header = cv_ptr_->header;
      hsv_ptr_->encoding = cv_ptr_->encoding;
      hsv_ptr_->image = object_.imgHSV_(rect);

      bgr_ptr_->header = cv_ptr_->header;
      bgr_ptr_->encoding = cv_ptr_->encoding;
      bgr_ptr_->image = cv_ptr_->image(rect);

      mono_ptr_->header = cv_ptr_->header;
      mono_ptr_->encoding = "mono8";
      cv::Mat ecc = cv::Mat::zeros(rect.height, rect.width, CV_8UC3);

      vector<cv::Point> poly;
      int angle = cvRound(rrect->angle);
      cv::ellipse2Poly(rrect->center - cv::Point2f(float(rect.x), float(rect.y)),
                       cv::Size(0.5*rrect->size.width, 0.5*rrect->size.height),
                       angle, 0, 355, 5, poly);
      //ROS_INFO_STREAM(poly.size());
      cv::fillConvexPoly(ecc, &poly[0], poly.size(), white);
      cv::cvtColor(ecc, mono_ptr_->image, CV_BGR2GRAY);

      hsv_image_pub_.publish(hsv_ptr_->toImageMsg());
      bgr_image_pub_.publish(bgr_ptr_->toImageMsg());
      mono_image_pub_.publish(mono_ptr_->toImageMsg());

      roi_pub_.publish(r);
      rrect_pub_.publish(rrect_msg);

    }


if ( tracked_image_pub_.getNumSubscribers() != 0)
  tracked_image_pub_.publish(cv_ptr_->toImageMsg());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracker_2d");
  Tracker2D tracker;
  ros::spin();
}
