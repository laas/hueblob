#ifndef HUEBLOB_HUEBLOB_H
# define HUEBLOB_HUEBLOB_H
# include <map>
# include <string>

# include <boost/noncopyable.hpp>
# include <opencv2/core/core.hpp>

# include <ros/ros.h>

// OpenCV bridge (OpenCV<->ROS conversion).
# include <cv_bridge/CvBridge.h>

// Image transport.
# include <image_transport/image_transport.h>
# include <image_transport/subscriber_filter.h>

// Message filters.
# include <message_filters/subscriber.h>
# include <message_filters/sync_policies/exact_time.h>
# include <message_filters/synchronizer.h>

// Messages.
# include <sensor_msgs/Image.h>
# include <stereo_msgs/DisparityImage.h>
# include "hueblob/Blob.h"
# include "hueblob/Blobs.h"

// Services.
# include "hueblob/AddObject.h"
# include "hueblob/ListObject.h"
# include "hueblob/RmObject.h"
# include "hueblob/TrackObject.h"


# include "libhueblob/object.hh"



/// \brief Main class of the HueBlob node.
///
/// This class is instantiated once by the main function.
class HueBlob : private boost::noncopyable
{
public:
  /// \name Constructors and destructors.
  /// \{

  /// \brief Construct and initialize the node.
  explicit HueBlob();
  virtual ~HueBlob();

  /// \}

  /// \brief Node main loop, only return when the node is terminating.
  void spin();

protected:
  /// \brief Reset the node to use a new stereo prefix.
  ///
  /// This allow run-time modification of the cameras used to track
  /// objects.
  //
  /// \param stereo_prefix topic name prefix for stereo cameras
  void setupInfrastructure(const std::string& stereo_prefix);

  /// \name Callbacks and services.
  /// \{

  /// \brief Image callback.
  ///
  /// Called when a synchronized triplet (left, right, disparity)
  /// has been received.
  void imageCallback(const sensor_msgs::ImageConstPtr& left,
		     const sensor_msgs::CameraInfoConstPtr& left_camera,
		     const sensor_msgs::ImageConstPtr& right,
		     const sensor_msgs::CameraInfoConstPtr& right_camera,
		     const stereo_msgs::DisparityImageConstPtr& disparity_msg);

  /// \brief AddObject service callback.
  ///
  /// This callback is run when an object is added to the object
  /// database (see objects_ attribute).
  bool AddObjectCallback(hueblob::AddObject::Request& request,
			 hueblob::AddObject::Response& response);

  /// \brief ListObject service callback.
  bool ListObjectCallback(hueblob::ListObject::Request& request,
			  hueblob::ListObject::Response& response);

  /// \brief RmObject service callback.
  bool RmObjectCallback(hueblob::RmObject::Request& request,
			hueblob::RmObject::Response& response);


  /// \brief TrackObject service callback.
  bool TrackObjectCallback(hueblob::TrackObject::Request& request,
			   hueblob::TrackObject::Response& response);

  /// \}

  /// \name Internal methods.
  /// \{

  void checkInputsSynchronized();

  hueblob::Blob trackBlob(const std::string&);

  /// \}

 private:
  /// \brief Define the synchronization policy.
  typedef message_filters::sync_policies::ExactTime<
   sensor_msgs::Image,
   sensor_msgs::CameraInfo,
   sensor_msgs::Image,
   sensor_msgs::CameraInfo,
   stereo_msgs::DisparityImage
   >
    SyncPolicy_t;

  /// \brief ROS node handle created at start-up.
  ros::NodeHandle nh_;
  /// \brief Image transport instanced used for image subscription/publishing.
  image_transport::ImageTransport it_;

  /// \name Parameters values
  /// \{

  /// \brief Stereo topic prefix.
  ///
  /// Stereo cameras topics are organized as follow:
  /// <prefix>/left/{image_raw, camera_info, ...}
  /// <prefix>/right/{image_raw, camera_info, ...}
  ///
  /// All topic names can be deduced from the prefix as long as this
  /// scheme is respected.
  std::string stereo_topic_prefix_;

  /// \brief ???
  double threshold_;

  /// \}

  /// \brief left image CvBridge.
  sensor_msgs::CvBridge bridgeLeft_;
  /// \brief disparity CvBridge.
  sensor_msgs::CvBridge bridgeDisparity_;


  /// \brief Left image subscriber.
  image_transport::SubscriberFilter left_sub_;
  /// \brief Left camera info subscriber.
  message_filters::Subscriber<sensor_msgs::CameraInfo> leftCamera_sub_;
  /// \brief Right image subscriber.
  image_transport::SubscriberFilter right_sub_;
  /// \brief Right camera info subscriber.
  message_filters::Subscriber<sensor_msgs::CameraInfo> rightCamera_sub_;
  /// \brief Disparity image subscriber.
  message_filters::Subscriber<stereo_msgs::DisparityImage> disparity_sub_;

  /// \brief Stereo information subscriber.
  ///
  /// This time synchronizer makes sure that both the left image,
  /// right image the disparity are received synchronously.
  message_filters::Synchronizer<SyncPolicy_t> sync_;

  /// \brief Blobs topic publisher.
  ///
  /// This topic provides information about tracked blob position
  /// asynchronously.
  ros::Publisher blobs_pub_;
  ros::Publisher cloud_pub_;
  /// Resulting tracked images
  image_transport::Publisher tracked_left_pub_;
  image_transport::Publisher tracked_right_pub_;

  /// \brief AddObject service server.
  ros::ServiceServer AddObject_srv_;

  /// \brief ListObject service server.
  ros::ServiceServer ListObject_srv_;

  /// \brief RmObject service server.
  ros::ServiceServer RmObject_srv_;

  /// \brief TrackObject service server.
  ros::ServiceServer TrackObject_srv_;

  /// \brief Object database.
  ///
  /// This associates each object name to its definition.
  std::map<std::string, Object> left_objects_;
  std::map<std::string, Object> right_objects_;

  /// \brief Timer used to periodically report bad synchronization.
  ros::WallTimer check_synced_timer_;

  /// \brief How many left images received so far?
  int left_received_;
  /// \brief How many right images received so far?
  int right_received_;
  /// \brief How many disparity images received so far?
  int disp_received_;
  /// \brief How many synchronized images received so far?
  int all_received_;

  /// \brief Last received image for the left camera.
  sensor_msgs::ImageConstPtr leftImage_;
  sensor_msgs::ImageConstPtr rightImage_;
  /// \brief Last received left camera info.
  sensor_msgs::CameraInfoConstPtr leftCamera_;
  /// \brief Last received disparity.
  stereo_msgs::DisparityImageConstPtr disparity_;

  /// blob dectection algorithm to be used
  std::string algo_;
  /// yaml filename that contains preloaded models
  std::string preload_models_;
};

#endif //! HUEBLOB_HUEBLOB_H
