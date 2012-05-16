#include "tracker_2d_nodelet.h"
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
#include <resource_retriever/retriever.h>

using namespace std;
namespace enc = sensor_msgs::image_encodings;

namespace hueblob {
  Tracker2DNodelet::Tracker2DNodelet()
    : nh_(),
      it_(nh_),
      sub_(),
      roi_pub_(),
      rrect_pub_(),
      tracked_image_pub_(),
      model_image_pub_(),
      image_(),
      model_path_(),
      name_(),
      object_(),
      cv_ptr_(),
      hsv_ptr_(new cv_bridge::CvImage),
      bgr_ptr_(new cv_bridge::CvImage),
      mono_ptr_(new cv_bridge::CvImage),
      model_ptr_(new cv_bridge::CvImage),
      new_model_ptr_(new cv_bridge::CvImage)
  {
  }

  void Tracker2DNodelet::onInit()
  {
    nh_ = getNodeHandle();
    it_ = image_transport::ImageTransport(nh_);
    object_ = Object();

    ros::NodeHandle local_nh = getPrivateNodeHandle();

    local_nh.param("image",  image_,
                   std::string("left/image_rect_color"));
    local_nh.param("name", name_,  std::string("rose"));
    local_nh.param("model", model_path_,
                   std::string("package://hueblob/data/models/ball-rose-3.png"));

    // Retrieve model image using resource retriever.
    resource_retriever::Retriever resourceRetriever;
    resource_retriever::MemoryResource resource =
      resourceRetriever.get(model_path_);
    std::vector<char> data;
    data.resize(resource.size);
    for (unsigned i = 0; i < resource.size; ++i)
      data[i] = resource.data[i];
    model_ptr_->image = cv::imdecode(cv::Mat (data), 1);
    if (!model_ptr_->image.data)
      throw std::runtime_error
	("failed to load the model image\n"
	 "please, double check the ~model parameter");

    ROS_INFO_STREAM("Loading " << model_path_ << " to object " << name_);
    object_.addView(model_ptr_->image);
    std::cout << image_ << name_;

    const::string image_topic         = ros::names::resolve(image_);
    const::string hint_topic          = ros::names::resolve("blobs/" + name_ + "/hint");
    const::string roi_topic           = ros::names::resolve("blobs/" + name_ + "/roi");
    const::string rrect_topic         = ros::names::resolve("blobs/" + name_ + "/rrect");
    const::string tracked_image_topic = ros::names::resolve("blobs/" + name_ + "/tracked_image");
    const::string model_image_topic   = ros::names::resolve("blobs/" + name_ + "/model_image");
    const::string new_model_image_topic   = ros::names::resolve("blobs/" + name_ + "/new_model_image");
    const::string hsv_image_topic     = ros::names::resolve("blobs/" + name_ + "/hsv_image");
    const::string bgr_image_topic     = ros::names::resolve("blobs/" + name_ + "/bgr_image");
    const::string mono_image_topic    = ros::names::resolve("blobs/" + name_ + "/mono_image");

    roi_pub_ = nh_.advertise<RoiStamped>(roi_topic, 5);
    rrect_pub_ = nh_.advertise<RotatedRectStamped>(rrect_topic, 5);
    tracked_image_pub_ = it_.advertise(tracked_image_topic, 1);
    model_image_pub_ = it_.advertise(model_image_topic, 1);
    hsv_image_pub_ = it_.advertise(hsv_image_topic, 1);
    bgr_image_pub_ = it_.advertise(bgr_image_topic, 1);
    mono_image_pub_ = it_.advertise(mono_image_topic, 1);

    sub_.subscribe(it_, image_topic, 5);
    sub_.registerCallback(boost::bind(&Tracker2DNodelet::imageCallback,
                                      this, _1));
    new_model_sub_.subscribe(it_, new_model_image_topic, 5);
    new_model_sub_.registerCallback(boost::bind(&Tracker2DNodelet::newModelCallback,
                                      this, _1));

    hint_sub_ = nh_.subscribe(hint_topic, 5,
                              &Tracker2DNodelet::hintCallback, this);
    ROS_INFO_STREAM(endl<< "Listening to:"
                    << "\n\t* " << image_topic
                    << "\n\t* " << hint_topic
                    << endl
                    << "Publishing to:"
                    << "\n\t* " << roi_topic
                    << "\n\t* " << rrect_topic
                    << "\n\t* " << tracked_image_topic
                    << "\n\t* " << hsv_image_topic
                    << "\n\t* " << bgr_image_topic
                    << "\n\t* " << mono_image_topic
                    << endl
                    );
  }

  void Tracker2DNodelet::hintCallback(const sensor_msgs::RegionOfInterestConstPtr& roi)
  {
    cv::Rect rect(roi->x_offset, roi->y_offset, roi->width, roi->height);
    object_.setSearchWindow(rect);
    // ROS_INFO_STREAM("Set rect " << rect.x
    //                 << " " << rect.y
    //                 << " " << rect.width
    //                 << " " << rect.height);
    return;
  }

  bool Tracker2DNodelet::rotated_rect(cv::Mat im, const cv::RotatedRect & rrect, cv::Scalar color)
  {
    // ROS_INFO_STREAM("INVALID RRECT " << rrect.center.x << " " << rrect.center.y << " "
    //                 << rrect.size.width << "  " << rrect.size.height << " "
    //                 << rrect.angle);
    CvPoint2D32f box_vtx[4];
    cvBoxPoints(rrect, box_vtx);
    bool res = true;
    cv::Point pt0, pt;
    pt0.x = cvRound(box_vtx[3].x);
    pt0.y = cvRound(box_vtx[3].y);

    if( (pt0.x <= 0) || (pt0.x >= im.cols)
        || (pt0.y <= 0) ||( pt0.y >= im.rows))
      res = false;

    for(int i = 0; i < 4; i++ )
      {
        pt.x = cvRound(box_vtx[i].x);
        pt.y = cvRound(box_vtx[i].y);
        if ( (pt.x <= 0) || (pt.x >= im.cols)
             || (pt.y <= 0) || (pt.y >= im.rows) )
          res = false;
        cv::line(im, pt0, pt, color, 1, CV_AA, 0);
        pt0 = pt;
      }
    return res;
  }

  void Tracker2DNodelet::draw_blob(cv::Mat im,
                                   const cv::RotatedRect &  rrect,
                                   const cv::Rect & rect,
                                   const std::string & name)
  {
    static const cv::Scalar color =  CV_RGB(255,0,0);
    static const cv::Scalar color2 = CV_RGB(0,0,255);
    static const cv::Scalar color3 = CV_RGB(0,255,0);
    cv::Point p1(rect.x, rect.y);
    cv::Point p2(rect.x + rect.width, rect.y + rect.height);
    cv::Point pc(rect.x, rect.y + std::max(16, rect.height+8));

    ROS_DEBUG_STREAM("Drawing rect " << rect.x << " "
                     << " " << rect.y
                     << " " << rect.width <<
                     " " << rect.height);
    cv::putText(im, name, p1,
                CV_FONT_HERSHEY_SIMPLEX,
                0.5, color);
    if (rotated_rect(im, rrect, color))
      {
        cv::ellipse(im, rrect, color2);
        cv::rectangle(im, p1, p2, color3);
      }

    if (!( 0 < rect.x && 0 <= rect.width
           && rect.x + rect.width < im.cols && 0 <= rect.y
           && 0 <= rect.height && rect.y + rect.height < im.rows ))
      {
        cv::Point p1(0, 20);
        cv::Point p2(0, 40);
        std::stringstream ss (std::stringstream::in
                              | std::stringstream::out);
        ss << "INVALID RRECT " << rrect.center.x << " "
           << rrect.center.y
           << " " << rrect.size.width << " " << rrect.size.height
           << " " << rrect.angle;
        static const cv::Scalar color =  CV_RGB(255,0,0);
        cv::putText(im, ss.str(), p1, CV_FONT_HERSHEY_SIMPLEX,
                    0.5, color);
        std::stringstream ss2 (std::stringstream::in
                               | std::stringstream::out);

        ss2 << "         RECT " << rect.x << " " << rect.y
            << " " << rect.width << " " << rect.height;
        cv::putText(im, ss2.str(), p2, CV_FONT_HERSHEY_SIMPLEX,
                    0.5, color);
      }
  }

  void Tracker2DNodelet::newModelCallback(const sensor_msgs::ImageConstPtr&
                                       msg)
  {
    try
      {
        new_model_ptr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    object_.clearViews();
    object_.addView(new_model_ptr_->image);
    model_ptr_->image = new_model_ptr_->image;
  }

  void Tracker2DNodelet::imageCallback(const sensor_msgs::ImageConstPtr&
                                       msg)
  {
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
    if ( model_image_pub_.getNumSubscribers() != 0)
      {
        model_ptr_->header = cv_ptr_->header;
        model_ptr_->encoding = cv_ptr_->encoding;
        model_image_pub_.publish(model_ptr_->toImageMsg());
      }


    boost::optional<cv::RotatedRect> rrect = object_.track(cv_ptr_->image);
    if (!rrect)
      {
        ROS_WARN_THROTTLE(20, "failed to track object");
        //ROS_WARN("failed to track object");
        RotatedRectStamped rrect_msg;
        rrect_msg.header = msg->header;
        rrect_pub_.publish(rrect_msg);
        return;
      }
    cv::Rect rect = rrect->boundingRect();

    RotatedRectStamped rrect_msg;
    rrect_msg.header = msg->header;
    rrect_msg.rrect.x = rrect->center.x;
    rrect_msg.rrect.y = rrect->center.y;
    rrect_msg.rrect.width = rrect->size.width;
    rrect_msg.rrect.height = rrect->size.height;
    rrect_msg.rrect.angle = rrect->angle;

    //ROS_WARN_STREAM("Publish" << r);
    rrect_pub_.publish(rrect_msg);

    if ( rrect && tracked_image_pub_.getNumSubscribers() != 0)
      {
        draw_blob(cv_ptr_->image, *rrect, rect, name_);
      }


    if (0 <= rect.x && 0 <= rect.width &&
        rect.x + rect.width < cv_ptr_->image.cols &&
        0 <= rect.y && 0 <= rect.height &&
        rect.y + rect.height < cv_ptr_->image.rows)
      {

        RoiStamped r;
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

        mono_ptr_->header = cv_ptr_->header;
        mono_ptr_->encoding = "mono8";
        cv::Mat ecc = cv::Mat::zeros(rect.height, rect.width, CV_8UC3);

        vector<cv::Point> poly;
        int angle = cvRound(rrect->angle);
        cv::ellipse2Poly(rrect->center - cv::Point2f(float(rect.x),
                                                     float(rect.y)),
                         cv::Size(0.5*rrect->size.width,
                                  0.5*rrect->size.height),
                         angle, 0, 355, 5, poly);
        //ROS_INFO_STREAM(poly.size());
        cv::fillConvexPoly(ecc, &poly[0], poly.size(), white);
        cv::cvtColor(ecc, mono_ptr_->image, CV_BGR2GRAY);

        roi_pub_.publish(r);
        hsv_image_pub_.publish(hsv_ptr_->toImageMsg());
        bgr_image_pub_.publish(bgr_ptr_->toImageMsg());
        mono_image_pub_.publish(mono_ptr_->toImageMsg());
      }



    if ( tracked_image_pub_.getNumSubscribers() != 0)
      tracked_image_pub_.publish(cv_ptr_->toImageMsg());
  }
} // namespace hueblob


#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(hueblob, tracker_2d,
                        hueblob::Tracker2DNodelet, nodelet::Nodelet)

