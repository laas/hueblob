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
  ros::Publisher hint_pub_;
  bool selecting_;
  std::string topic_;
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  std::string transport_;

  virtual void onInit();
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  static void trackButtonCb(GtkWidget *widget,
                            gpointer   data );

  static void mouseCb(int event, int x, int y, int flags, void* param);


public:
  MonitorNodelet();

  ~MonitorNodelet();
};

MonitorNodelet::MonitorNodelet()
  : filename_format_(""),
    nh_(),
    it_(nh_),
    count_(0),
    clicked_p_(),
    pressed_p_(),
    selecting_(false),
    topic_()
{
}

MonitorNodelet::~MonitorNodelet()
{
  cv::destroyWindow(window_name_);
}

void MonitorNodelet::onInit()
{
  NODELET_DEBUG("Initializing nodelet");

  nh_ = getNodeHandle();
  ros::NodeHandle local_nh = getPrivateNodeHandle();
  // Command line argument parsing
  const std::vector<std::string>& argv = getMyArgv();
  // First positional argument is the transport type
  transport_ = "raw";
  for (int i = 0; i < (int)argv.size(); ++i)
  {
    if (argv[i][0] != '-')
    {
      transport_ = argv[i];
      break;
    }
  }
  // Internal option, should be used only by the hueblob node
  bool shutdown_on_close = std::find(argv.begin(), argv.end(),
                                     "--shutdown-on-close") != argv.end();

  // Default window name is the resolved topic name
  topic_ = nh_.resolveName("image");
  local_nh.param("window_name", window_name_, topic_);

  bool autosize;
  local_nh.param("autosize", autosize, false);

  std::string format_string;
  local_nh.param("filename_format", format_string, std::string("frame%04i.jpg"));
  filename_format_.parse(format_string);

  std::string hint_topic = nh_.resolveName("hint");
  hint_pub_ = local_nh.advertise<sensor_msgs::RegionOfInterest>(hint_topic, 1);

  cv::namedWindow(window_name_, autosize ? CV_WINDOW_AUTOSIZE : 0);
  cv::setMouseCallback(window_name_, &MonitorNodelet::mouseCb, this);

#ifdef HAVE_GTK
  // Register appropriate handler for when user closes the display window
  GtkWidget *control_w = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(control_w), "Monitor");
  gtk_window_set_default_size (GTK_WINDOW(control_w), 200,500);
  GtkWidget *vbox = gtk_vbox_new(false, 0);
  gtk_container_add (GTK_CONTAINER (control_w), vbox);

  GtkWidget *hbox_top = gtk_hbox_new(false, 0);
  gtk_box_pack_start(GTK_BOX(vbox), hbox_top, false, false, 0);

  GtkWidget *track_button = gtk_check_button_new_with_label("Track");
  gtk_box_pack_start(GTK_BOX(hbox_top), track_button, false, false, 0);
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(track_button), true);
  g_signal_connect (track_button, "toggled",
                    G_CALLBACK (&MonitorNodelet::trackButtonCb), this);

  gtk_widget_show_all (control_w);
  GtkWidget *image_w = GTK_WIDGET( cvGetWindowHandle(window_name_.c_str()) );
  if (shutdown_on_close)
    {
      g_signal_connect(image_w, "destroy", G_CALLBACK(destroyNode), NULL);
      g_signal_connect(control_w, "destroy", G_CALLBACK(destroyNode), NULL);
    }
  else
    {
      g_signal_connect(image_w, "destroy", G_CALLBACK(destroyNodelet), &sub_);
      g_signal_connect(control_w, "destroy", G_CALLBACK(destroyNodelet), &sub_);
    }

#endif

  // Start the OpenCV window thread so we don't have to waitKey() somewhere
  startWindowThread();

  it_ = image_transport::ImageTransport(nh_);
  sub_ = it_.subscribe(topic_, 1, &MonitorNodelet::imageCb, this, transport_);
}

void MonitorNodelet::trackButtonCb(GtkWidget *widget,
                                   gpointer   data
                                   )
{
  bool state =  gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(widget));
  MonitorNodelet *this_ = reinterpret_cast<MonitorNodelet*>(data);
  if (!state)
    {
      this_->sub_.shutdown();
    }

  else if (state)
    {
      this_->sub_ = this_->it_.subscribe(this_->topic_, 1, &MonitorNodelet::imageCb, this_,
                                 this_->transport_);
    }
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
    if (selecting_)
      {
        static const cv::Scalar color = CV_RGB(0,255,0);
        cv::rectangle(last_image_, clicked_p_, pressed_p_, color, 1);
      }
    cv::imshow(window_name_, last_image_);
}

void MonitorNodelet::mouseCb(int event, int x, int y, int flags, void* param)
{
  MonitorNodelet *this_ = reinterpret_cast<MonitorNodelet*>(param);

  if  (this_->selecting_)
    {
      this_->pressed_p_.x = x;
      this_->pressed_p_.y = y;
    }

  // Trick to use NODELET_* logging macros in static function
  boost::function<const std::string&()> getName =
    boost::bind(&MonitorNodelet::getName, this_);
  if (event == CV_EVENT_LBUTTONDOWN)
  {
    NODELET_WARN_ONCE("Left-clicking no longer saves images. Right-click instead.");
    this_->selecting_ = ! this_->selecting_;

    if (this_->selecting_)
      {
        this_->clicked_p_.x = x;
        this_->clicked_p_.y = y;
      }
    else
      {
        sensor_msgs::RegionOfInterest msg;
        msg.x_offset = std::min(this_->clicked_p_.x, this_->pressed_p_.x);
        msg.y_offset = std::min(this_->clicked_p_.y, this_->pressed_p_.y);
        msg.width    = std::abs(this_->clicked_p_.x - this_->pressed_p_.x);
        msg.height   = std::abs(this_->clicked_p_.y - this_->pressed_p_.y);
        this_->hint_pub_.publish(msg);
      }
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
