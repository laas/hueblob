#include <ros/ros.h>
#include <ros/console.h>
#include "libhueblob/object.hh"
#include <sensor_msgs/Image.h>
#include "cv.h"
#include "highgui.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <hueblob/Box.h>

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
  ros::Publisher pub_;
  string image_, model_path_, name_;
  Object object_;
};


Tracker2D::Tracker2D()
  : nh_("track_2d"),
    it_(nh_),
    sub_(),
    pub_(),
    image_(),
    model_path_(),
    name_(),
    object_()
{

  ros::param::param<string>("~image", image_, "left/image_rect_color");
  ros::param::param<string>("~name", name_, "test");
  ros::param::param<string>("~model", model_path_, "/home/nddang/src/ros/hueblob/data/models/ball-rose-5.png");

  const::string image_topic = ros::names::resolve(image_);
  cv::Mat model = cv::imread(model_path_.c_str());
  object_.addView(model);

  const::string box_topic = ros::names::resolve("blobs/" + name_ + "/blob2d");
  pub_ = nh_.advertise<hueblob::Box>(box_topic, 5);

  sub_.subscribe(it_, image_topic, 5);
  sub_.registerCallback(boost::bind(&Tracker2D::imageCallback,
                                    this, _1));

  ROS_INFO_STREAM("Listening to " << image_topic);

}

void Tracker2D::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  boost::optional<cv::RotatedRect> rrect = object_.track(cv_ptr->image);
  if (!rrect)
    {
      ROS_WARN_THROTTLE(20, "failed to track object");
      return;
    }
  cv::Rect rect = rrect->boundingRect();
  hueblob::Box box;
  box.header = msg->header;
  box.x = rect.x;
  box.y = rect.y;
  box.width = rect.width;
  box.height = rect.height;
  pub_.publish(box);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracker_2d");
  Tracker2D tracker;
  ros::spin();
}
