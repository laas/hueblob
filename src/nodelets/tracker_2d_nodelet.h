#ifndef __TRACKER_2D_NODELET__
#define __TRACKER_2D_NODELET__

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

#include <nodelet/nodelet.h>


namespace hueblob {
  class Tracker2DNodelet : public nodelet::Nodelet
    {
    public:
      Tracker2DNodelet();
      virtual ~Tracker2DNodelet(){};
      static void draw_blob(cv::Mat im, const cv::RotatedRect &  rrect,
                                   const cv::Rect & rect,
                                      const std::string & name);
    private:
      void imageCallback(const sensor_msgs::ImageConstPtr& image);
      void newModelCallback(const sensor_msgs::ImageConstPtr& image);
      void hintCallback(const sensor_msgs::RegionOfInterestConstPtr& roi);
      virtual void onInit();


      static bool rotated_rect(cv::Mat im, const cv::RotatedRect & rrect, cv::Scalar color);

      ros::NodeHandle nh_;
      image_transport::ImageTransport it_;
      image_transport::SubscriberFilter sub_, new_model_sub_;
      ros::Subscriber hint_sub_;
      ros::Publisher roi_pub_, rrect_pub_;
      image_transport::Publisher tracked_image_pub_, model_image_pub_ ;
      image_transport::Publisher hsv_image_pub_, bgr_image_pub_, mono_image_pub_;
      std::string image_, model_path_, name_;
      Object object_;
      cv_bridge::CvImagePtr cv_ptr_, hsv_ptr_, bgr_ptr_, mono_ptr_, model_ptr_, new_model_ptr_;
    };
}

#endif
