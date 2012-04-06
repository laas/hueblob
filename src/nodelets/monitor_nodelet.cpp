#include "tracker_2d_nodelet.h"
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include "window_thread.h"

#include <boost/thread.hpp>
#include <boost/format.hpp>

// Message filters.
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#ifdef HAVE_GTK
#include <gtk/gtk.h>
#include <cv.h>
#include <message_filters/time_synchronizer.h>
#include <ros/package.h>
#include <fstream>

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

using namespace message_filters;
namespace enc = sensor_msgs::image_encodings;

namespace hueblob {

class MonitorNodelet : public nodelet::Nodelet
{
  typedef sync_policies::ExactTime<sensor_msgs::Image, RotatedRectStamped> Policy;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  //sensor_msgs::CvBridge img_bridge_, model_bridge_;

  boost::mutex monitor_mutex_;
  cv::Mat last_image_, model_image_, new_model_image_;

  std::string window_name_;
  boost::format filename_format_;
  int count_;

  cv::Point clicked_p_;
  cv::Point pressed_p_;
  ros::Publisher hint_pub_;
  image_transport::Publisher new_model_pub_;
  bool selecting_;
  bool draw_blob_;
  bool track_;
  std::string topic_;
  std::string rrect_topic_;
  std::string model_topic_, new_model_topic_;
  std::string transport_;
  std::string blob_name_;
  GtkWidget *model_draw_area_, *new_model_draw_area_;
  image_transport::SubscriberFilter image_sub_, model_sub_;
  message_filters::Subscriber<RotatedRectStamped> rrect_sub_;
  sensor_msgs::ImageConstPtr last_msg_, model_msg_, last_model_msg_;
  cv_bridge::CvImagePtr im_ptr_, model_ptr_, new_model_ptr_;

  Synchronizer<Policy> sync_;

  virtual void onInit();
  void callback(const sensor_msgs::ImageConstPtr& msg,
                const RotatedRectStampedConstPtr& rrect );
  void modelCallback(const sensor_msgs::ImageConstPtr& msg);
  static void trackButtonCb(GtkWidget *widget, gpointer   data );
  static void sendButtonCb(GtkWidget *widget, gpointer   data );
  static void saveButtonCb(GtkWidget *widget, gpointer   data );
  static void drawBlobButtonCb(GtkWidget *widget, gpointer   data );
  static gboolean modelAreaCb(GtkWidget *widget, GdkEventExpose *event, gpointer   data );
  static gboolean newModelAreaCb(GtkWidget *widget, GdkEventExpose *event, gpointer   data );
  static void mouseCb(int event, int x, int y, int flags, void* param);


public:
  MonitorNodelet();

  ~MonitorNodelet();
};

MonitorNodelet::MonitorNodelet()
  : nh_(),
    it_(nh_),
    filename_format_(""),
    count_(0),
    clicked_p_(),
    pressed_p_(),
    selecting_(false),
    draw_blob_(true),
    track_(true),
    im_ptr_(),
    model_ptr_(),
    new_model_ptr_(new cv_bridge::CvImage),
    sync_(10)
{
}

MonitorNodelet::~MonitorNodelet()
{
  cv::destroyWindow(window_name_);
}

void MonitorNodelet::onInit()
{
  NODELET_DEBUG("Initializing nodelet");
  new_model_ptr_->encoding = "bgr8";
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


  local_nh.param("name", blob_name_,  std::string("rose"));
  local_nh.param("image" , topic_, std::string("left/image_rect_color"));

  topic_         = ros::names::resolve(topic_);

  local_nh.param("window_name", window_name_, topic_);

  bool autosize;
  local_nh.param("autosize" , autosize, false);

  std::string format_string;
  local_nh.param("filename_format", format_string, std::string("frame%04i.jpg"));
  filename_format_.parse(format_string);

  std::string hint_topic = ros::names::resolve("blobs/" + blob_name_ + "/hint");
  hint_pub_ = local_nh.advertise<sensor_msgs::RegionOfInterest>(hint_topic, 1);
  new_model_topic_ = ros::names::resolve("blobs/" + blob_name_ + "/new_model_image");
  new_model_pub_ = it_.advertise(new_model_topic_, 1);

  cv::namedWindow(window_name_, autosize ? CV_WINDOW_AUTOSIZE : 0);
  cv::setMouseCallback(window_name_, &MonitorNodelet::mouseCb, this);

#ifdef HAVE_GTK
  // Register appropriate handler for when user closes the display window
  GtkWidget *control_w = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title(GTK_WINDOW(control_w), "Monitor");
  //gtk_window_set_default_size (GTK_WINDOW(control_w), 200,500);
  GtkWidget *vbox = gtk_vbox_new(false, 0);
  gtk_container_add (GTK_CONTAINER (control_w), vbox);

  GtkWidget *hbox_top = gtk_hbox_new(false, 0);
  gtk_box_pack_start(GTK_BOX(vbox), hbox_top, false, false, 0);

  GtkWidget *track_button = gtk_check_button_new_with_label("Track");
  gtk_box_pack_start(GTK_BOX(hbox_top), track_button, false, false, 0);
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(track_button), true);
  g_signal_connect (track_button, "toggled",
                    G_CALLBACK (&MonitorNodelet::trackButtonCb), this);

  GtkWidget *draw_blob_button = gtk_check_button_new_with_label("Draw blob");
  gtk_box_pack_start(GTK_BOX(hbox_top), draw_blob_button, false, false, 0);
  gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(draw_blob_button), true);
  g_signal_connect (draw_blob_button, "toggled",
                    G_CALLBACK (&MonitorNodelet::drawBlobButtonCb), this);

  model_draw_area_ = gtk_drawing_area_new ();
  gtk_widget_set_size_request (model_draw_area_, 200, 200);
  g_signal_connect (G_OBJECT (model_draw_area_), "expose_event",
                    G_CALLBACK (&MonitorNodelet::modelAreaCb), this);
  GtkWidget* model_frame = gtk_frame_new ("Current model");
  gtk_container_add (GTK_CONTAINER (model_frame), model_draw_area_);
  gtk_box_pack_start(GTK_BOX(vbox), model_frame, false, false, 0);


  GtkWidget *vsep = gtk_vseparator_new();
  gtk_box_pack_start(GTK_BOX(vbox), vsep, false, false, 0);

  new_model_draw_area_ = gtk_drawing_area_new ();
  gtk_widget_set_size_request (new_model_draw_area_, 200, 200);
  g_signal_connect (G_OBJECT (new_model_draw_area_), "expose_event",
                    G_CALLBACK (&MonitorNodelet::newModelAreaCb), this);
  GtkWidget* new_model_frame = gtk_frame_new ("Selected");
  gtk_container_add (GTK_CONTAINER (new_model_frame), new_model_draw_area_);
  gtk_box_pack_start(GTK_BOX(vbox), new_model_frame, false, false, 0);


  GtkWidget *bottom_hbox = gtk_hbox_new(false, 0);
  gtk_box_pack_start(GTK_BOX(vbox), bottom_hbox, false, false, 0);

  GtkWidget *send_button = gtk_button_new_with_label("Send as model");
  gtk_box_pack_start(GTK_BOX(bottom_hbox), send_button, false, false, 0);
  g_signal_connect (G_OBJECT (send_button), "clicked",
                    G_CALLBACK (&MonitorNodelet::sendButtonCb), this);

  GtkWidget *save_button = gtk_button_new_with_label("Save config");
  gtk_box_pack_start(GTK_BOX(bottom_hbox), save_button, false, false, 0);
  g_signal_connect (G_OBJECT (save_button), "clicked",
                    G_CALLBACK (&MonitorNodelet::saveButtonCb), this);




  gtk_widget_show_all (control_w);
  GtkWidget *image_w = GTK_WIDGET( cvGetWindowHandle(window_name_.c_str()) );
  if (shutdown_on_close)
    {
      g_signal_connect(image_w, "destroy", G_CALLBACK(destroyNode), NULL);
      g_signal_connect(control_w, "destroy", G_CALLBACK(destroyNode), NULL);
    }
  else
    {
      g_signal_connect(image_w, "destroy", G_CALLBACK(destroyNodelet), &image_sub_);
      g_signal_connect(control_w, "destroy", G_CALLBACK(destroyNodelet), &image_sub_);
    }

#endif

  // Start the OpenCV window thread so we don't have to waitKey() somewhere
  startWindowThread();

  rrect_topic_ = ros::names::resolve("blobs/" + blob_name_ + "/rrect");
  model_topic_ = ros::names::resolve("blobs/" + blob_name_ + "/model_image");
  it_ = image_transport::ImageTransport(nh_);
  image_sub_.subscribe(it_, topic_, 10);
  rrect_sub_.subscribe(nh_, rrect_topic_, 10);
  model_sub_.subscribe(it_, model_topic_, 10);
  model_sub_.registerCallback(boost::bind(&MonitorNodelet::modelCallback,
                                     this, _1));
  sync_.connectInput(image_sub_, rrect_sub_);
  sync_.registerCallback(boost::bind(&MonitorNodelet::callback,
                                     this, _1, _2));
}

  void MonitorNodelet::modelCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    //    monitor_mutex_.lock();
    model_msg_ = msg;
    if (!last_model_msg_)
      last_model_msg_ = msg;

    try
      {
        model_ptr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }

    model_image_ = model_ptr_->image;
    gtk_widget_queue_draw( GTK_WIDGET(model_draw_area_) );
    //monitor_mutex_.unlock();
  }


gboolean MonitorNodelet::modelAreaCb(GtkWidget *widget,GdkEventExpose *event,
                                 gpointer   data
                                 )
{
  MonitorNodelet *this_ = reinterpret_cast<MonitorNodelet*>(data);
  if (this_->model_image_.rows == 0)
    {
      return TRUE;
    }

  cv::Mat im;
  cv::cvtColor(this_->model_image_, im, CV_BGR2RGB);

  cv::Size size;
  if (im.rows > im.cols)
    {
      size = cv::Size( cvRound( (float)(im.cols)/(float)(im.rows)*200.0 ), 200);
    }
  else
    {
      size = cv::Size(200, cvRound( (float)(im.rows)/(float)(im.cols)*200.0));
    }
  cv::resize(im, im, size, 0, 0, cv::INTER_NEAREST);


  gdk_window_clear_area (widget->window,
                         0, 0,
                         widget->allocation.width,
                         widget->allocation.height);
  int x0 = (widget->allocation.width - im.cols)/2;
  int y0 = (widget->allocation.height - im.rows)/2;


  gdk_draw_rgb_image( widget->window, widget->style->fg_gc[GTK_STATE_NORMAL],
                      x0, y0, MIN(im.cols, widget->allocation.width),
                      MIN(im.rows, widget->allocation.height),
                      GDK_RGB_DITHER_MAX, im.data, im.step );
  this_->last_model_msg_ = this_->model_msg_;
  return TRUE;
}


gboolean MonitorNodelet::newModelAreaCb(GtkWidget *widget,GdkEventExpose *event,
                                 gpointer   data
                                 )
{
  MonitorNodelet *this_ = reinterpret_cast<MonitorNodelet*>(data);
  if (this_->new_model_image_.empty())
    {
      return TRUE;
    }

  cv::Mat im;
  cv::cvtColor(this_->new_model_image_, im, CV_BGR2RGB);
  cv::Size size;
  if (im.rows > im.cols)
    {
      size = cv::Size(cvRound( (float)(im.cols)/(float)(im.rows)*200.0), 200 );
    }
  else
    {
      size = cv::Size(200, cvRound( (float)(im.rows)/(float)(im.cols)*200.0 ));
    }
  cv::resize(im, im, size, 0, 0, cv::INTER_NEAREST);


  gdk_window_clear_area (widget->window,
                         0, 0,
                         widget->allocation.width,
                         widget->allocation.height);
  int x0 = (widget->allocation.width - im.cols)/2;
  int y0 = (widget->allocation.height - im.rows)/2;


  gdk_draw_rgb_image( widget->window, widget->style->fg_gc[GTK_STATE_NORMAL],
                      x0, y0, MIN(im.cols, widget->allocation.width),
                      MIN(im.rows, widget->allocation.height),
                      GDK_RGB_DITHER_MAX, im.data, im.step );
  this_->new_model_ptr_->image = this_->new_model_image_;
  return TRUE;
}



void MonitorNodelet::drawBlobButtonCb(GtkWidget *widget, gpointer   data)
{

  MonitorNodelet *this_ = reinterpret_cast<MonitorNodelet*>(data);
  //this_->monitor_mutex_.lock();

  this_->draw_blob_ =  gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(widget));
  //this_->monitor_mutex_.unlock();

}
  namespace{
    bool fexists(const char *filename)
    {
      std::ifstream ifile(filename);
      return ifile;
    }
  }

void MonitorNodelet::saveButtonCb(GtkWidget *widget, gpointer   data)
{
  MonitorNodelet *this_ = reinterpret_cast<MonitorNodelet*>(data);
  static std::string path = ros::package::getPath("hueblob");
  static unsigned count(0);
  std::string filename("foo");
  bool fexist(true);

  while (fexist){
    filename = (boost::format("%s/data/models/%s_%04i.png")
                            % (path)
                            % (this_->blob_name_)
                            % count ).str();
    fexist = fexists(filename.c_str());

    count ++;
  }
  ROS_INFO_STREAM("Saving model to " << filename << std::endl);
  cv::imwrite(filename, this_->model_image_);

  std::string config = (boost::format("<launch>\n"
                                      "<param name='/wide/tracker_2d/model' value='%s'/>\n"
                                      "</launch>\n"
                                      ) % filename).str();
  std::string config_filename = (boost::format("%s/launch/config.launch") %path).str();
  std::ofstream outfile (config_filename.c_str());
  outfile << config;
  outfile.close();

}

void MonitorNodelet::sendButtonCb(GtkWidget *widget,gpointer   data)
{
  MonitorNodelet *this_ = reinterpret_cast<MonitorNodelet*>(data);
  //this_->monitor_mutex_.lock();

  static bool once(false);
  if (!once)
    {
      once = true;
      this_->new_model_ptr_->header = this_->model_ptr_->header;
    }

  if (!this_->new_model_ptr_->image.empty())
    {
      this_->new_model_pub_.publish(this_->new_model_ptr_->toImageMsg());
    }
  //this_->monitor_mutex_.unlock();
}
void MonitorNodelet::trackButtonCb(GtkWidget *widget,gpointer   data)
{

  MonitorNodelet *this_ = reinterpret_cast<MonitorNodelet*>(data);
  //  this_->monitor_mutex_.lock();

  this_->track_ =  gtk_toggle_button_get_active (GTK_TOGGLE_BUTTON(widget));
  if (!this_->track_)
    {
      this_->image_sub_.unsubscribe();
      this_->rrect_sub_.unsubscribe();
    }

  else if (this_->track_)
    {
      this_->image_sub_.subscribe(this_->it_, this_->topic_, 10);
      this_->rrect_sub_.subscribe(this_->nh_, this_->rrect_topic_, 10);
    }
  //  this_->monitor_mutex_.unlock();

}


  void MonitorNodelet::callback(const sensor_msgs::ImageConstPtr& msg,
                             const RotatedRectStampedConstPtr& rrect_msg)
{
  monitor_mutex_.lock();

  // May want to view raw bayer data, which CvBridge doesn't know about

    // Convert to OpenCV native BGR color

  try
    {
      im_ptr_ = cv_bridge::toCvCopy(msg, enc::BGR8);
      last_image_ = im_ptr_->image;
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }


  // last_image_ may point to data owned by last_msg_, so we hang onto it for
  // the sake of mouseCb.
  last_msg_ = msg;

  // Must release the mutex before calling cv::imshow, or can deadlock against
  // OpenCV's window mutex.

  cv::RotatedRect rrect;
  rrect.center.x = rrect_msg->rrect.x;
  rrect.center.y = rrect_msg->rrect.y;
  rrect.size.width = rrect_msg->rrect.width;
  rrect.size.height = rrect_msg->rrect.height;
  rrect.angle = rrect_msg->rrect.angle;

  cv::Rect rect = rrect.boundingRect();

  if (draw_blob_)
    {
      Tracker2DNodelet::draw_blob(last_image_, rrect, rect, blob_name_);
    }

  monitor_mutex_.unlock();
  if (!last_image_.empty())
      if (selecting_ && clicked_p_.x != 0 && clicked_p_.y != 0
          && pressed_p_.x != 0 && pressed_p_.y != 0)
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
      if (!this_->track_ && this_->clicked_p_.x != 0
          && this_->clicked_p_.y != 0)
        {
          cv::Mat im;
          this_->last_image_.copyTo(im);
          static const cv::Scalar color = CV_RGB(0,255,0);
          cv::rectangle(im, this_->clicked_p_, this_->pressed_p_, color, 1);
          cv::imshow(this_->window_name_, im);
        }
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
        cv::Rect rect;
        rect.x = std::min(this_->clicked_p_.x, this_->pressed_p_.x) + 1;
        rect.y = std::min(this_->clicked_p_.y, this_->pressed_p_.y) + 1;
        rect.width    = std::abs(this_->clicked_p_.x - this_->pressed_p_.x) - 1;
        rect.height   = std::abs(this_->clicked_p_.y - this_->pressed_p_.y) - 1;
        if (rect.width >0 && rect.height >0)
          {
            this_->new_model_image_ = this_->last_image_(rect);
            gtk_widget_queue_draw( GTK_WIDGET(this_->new_model_draw_area_) );
            this_->hint_pub_.publish(msg);
            this_->clicked_p_.x = this_->clicked_p_.y = this_->pressed_p_.x = this_->pressed_p_.y = 0.;
          }
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
