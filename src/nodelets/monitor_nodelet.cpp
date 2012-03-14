#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/CvBridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "window_thread.h"

#include <boost/thread.hpp>
#include <boost/format.hpp>

#ifdef HAVE_GTK
#include <gtk/gtk.h>
#include <cv.h>
static void destroyNode(GtkWidget *widget, gpointer data)
{
  /// @todo On ros::shutdown(), the node hangs. Why?
  //ros::shutdown();
  exit(0); // brute force solution
}

static void destroyNodelet(GtkWidget *widget, gpointer data)
{
  // We can't actually unload the nodelet from here, but we can at least
  // unsubscribe from the image topic.
  reinterpret_cast<image_transport::Subscriber*>(data)->shutdown();
}
#endif


namespace hueblob {

class MonitorNodelet : public nodelet::Nodelet
{
  image_transport::Subscriber sub_;
  sensor_msgs::CvBridge img_bridge_;

  boost::mutex monitor_mutex_;
  sensor_msgs::ImageConstPtr last_msg_;
  cv::Mat last_image_;

  std::string window_name_;
  boost::format filename_format_;
  int count_;

  cv::Point clicked_p_;
  cv::Point pressed_p_;
  cv::Rect selected_rect_;
  ros::Publisher hint_pub_;

  virtual void onInit();

  void imageCb(const sensor_msgs::ImageConstPtr& msg);

  static void mouseCb(int event, int x, int y, int flags, void* param);

public:
  MonitorNodelet();

  ~MonitorNodelet();
};

MonitorNodelet::MonitorNodelet()
  : filename_format_(""), count_(0), clicked_p_(), pressed_p_(), selected_rect_()
{
}

MonitorNodelet::~MonitorNodelet()
{
  cv::destroyWindow(window_name_);
}

void MonitorNodelet::onInit()
{
  NODELET_DEBUG("Initializing nodelet");

  ros::NodeHandle nh = getNodeHandle();
  ros::NodeHandle local_nh = getPrivateNodeHandle();
  // Command line argument parsing
  const std::vector<std::string>& argv = getMyArgv();
  // First positional argument is the transport type
  std::string transport = "raw";
  for (int i = 0; i < (int)argv.size(); ++i)
  {
    if (argv[i][0] != '-')
    {
      transport = argv[i];
      break;
    }
  }
  // Internal option, should be used only by the hueblob node
  bool shutdown_on_close = std::find(argv.begin(), argv.end(),
                                     "--shutdown-on-close") != argv.end();

  // Default window name is the resolved topic name
  std::string topic = nh.resolveName("image");
  local_nh.param("window_name", window_name_, topic);

  bool autosize;
  local_nh.param("autosize", autosize, false);

  std::string format_string;
  local_nh.param("filename_format", format_string, std::string("frame%04i.jpg"));
  filename_format_.parse(format_string);

  std::string hint_topic = nh.resolveName("hint");
  hint_pub_ = local_nh.advertise<sensor_msgs::RegionOfInterest>(hint_topic, 1);

  cv::namedWindow(window_name_, autosize ? CV_WINDOW_AUTOSIZE : 0);
  cv::setMouseCallback(window_name_, &MonitorNodelet::mouseCb, this);

#ifdef HAVE_GTK
  // Register appropriate handler for when user closes the display window
  GtkWidget *widget = GTK_WIDGET( cvGetWindowHandle(window_name_.c_str()) );
  if (shutdown_on_close)
    g_signal_connect(widget, "destroy", G_CALLBACK(destroyNode), NULL);
  else
    g_signal_connect(widget, "destroy", G_CALLBACK(destroyNodelet), &sub_);
#endif

  // Start the OpenCV window thread so we don't have to waitKey() somewhere
  startWindowThread();

  image_transport::ImageTransport it(nh);
  sub_ = it.subscribe(topic, 1, &MonitorNodelet::imageCb, this, transport);
}

void MonitorNodelet::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  monitor_mutex_.lock();

  // May want to view raw bayer data, which CvBridge doesn't know about
  if (msg->encoding.find("bayer") != std::string::npos)
  {
    last_image_ = cv::Mat(msg->height, msg->width, CV_8UC1,
                          const_cast<uint8_t*>(&msg->data[0]), msg->step);
  }
  // We want to scale floating point images so that they display nicely
  else if(msg->encoding.find("F") != std::string::npos)
  {
    cv::Mat float_image_bridge = img_bridge_.imgMsgToCv(msg, "passthrough");
    cv::Mat_<float> float_image = float_image_bridge;
    float max_val = 0;
    for(int i = 0; i < float_image.rows; ++i)
    {
      for(int j = 0; j < float_image.cols; ++j)
      {
        max_val = std::max(max_val, float_image(i, j));
      }
    }

    if(max_val > 0)
    {
      float_image /= max_val;
    }
    last_image_ = float_image;
  }
  else
  {
    // Convert to OpenCV native BGR color
    try {
      last_image_ = img_bridge_.imgMsgToCv(msg, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException& e) {
      NODELET_ERROR_THROTTLE(30, "Unable to convert '%s' image to bgr8",
                             msg->encoding.c_str());
    }
  }

  // last_image_ may point to data owned by last_msg_, so we hang onto it for
  // the sake of mouseCb.
  last_msg_ = msg;

  // Must release the mutex before calling cv::imshow, or can deadlock against
  // OpenCV's window mutex.
  monitor_mutex_.unlock();
  if (!last_image_.empty())
    if (selected_rect_.width && selected_rect_.height)
      {
        static const cv::Scalar color = CV_RGB(0,255,0);
        cv::rectangle(last_image_, clicked_p_, pressed_p_, color, 1);
      }
    cv::imshow(window_name_, last_image_);
}

void MonitorNodelet::mouseCb(int event, int x, int y, int flags, void* param)
{
  MonitorNodelet *this_ = reinterpret_cast<MonitorNodelet*>(param);
  // Trick to use NODELET_* logging macros in static function
  boost::function<const std::string&()> getName =
    boost::bind(&MonitorNodelet::getName, this_);

  if (event == CV_EVENT_LBUTTONDOWN)
  {
    NODELET_WARN_ONCE("Left-clicking no longer saves images. Right-click instead.");
    this_->clicked_p_.x = x;
    this_->clicked_p_.y = y;
    return;
  }
  else if (event == CV_EVENT_LBUTTONUP)
    {
      this_->clicked_p_ = cv::Point();
      this_->pressed_p_ = cv::Point();
      this_->selected_rect_ = cv::Rect();
    }

  else if (flags == CV_EVENT_FLAG_LBUTTON)
    {
      this_->pressed_p_.x = x;
      this_->pressed_p_.y = y;

      this_->selected_rect_.x = std::min(this_->clicked_p_.x, this_->pressed_p_.x);
      this_->selected_rect_.y = std::min(this_->clicked_p_.y, this_->pressed_p_.y);
      this_->selected_rect_.width  = std::abs(this_->clicked_p_.x - this_->pressed_p_.x);
      this_->selected_rect_.height = std::abs(this_->clicked_p_.y - this_->pressed_p_.y);

      sensor_msgs::RegionOfInterest msg;
      msg.x_offset = this_->selected_rect_.x;
      msg.y_offset = this_->selected_rect_.y;
      msg.width    = this_->selected_rect_.width;
      msg.height   = this_->selected_rect_.height;

      this_->hint_pub_.publish(msg);

    }

  if (event != CV_EVENT_RBUTTONDOWN)
    return;

  boost::lock_guard<boost::mutex> guard(this_->monitor_mutex_);

  const cv::Mat image = this_->last_image_;
  if (image.empty())
  {
    NODELET_WARN("Couldn't save image, no data!");
    return;
  }

  std::string filename = (this_->filename_format_ % this_->count_).str();
  if (cv::imwrite(filename, image))
  {
    NODELET_INFO("Saved image %s", filename.c_str());
    this_->count_++;
  }
  else
  {
    /// @todo Show full path, ask if user has permission to write there
    NODELET_ERROR("Failed to save image.");
  }
}

} // namespace hueblob

// Register the nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS(hueblob, monitor, hueblob::MonitorNodelet, nodelet::Nodelet)
