#ifndef HUEBLOB_HUEBLOB_H
# define HUEBLOB_HUEBLOB_H
# include <map>
# include <string>

# include <boost/noncopyable.hpp>

# include <ros/ros.h>

# include <cv_bridge/CvBridge.h>
# include <image_transport/image_transport.h>
# include <image_transport/subscriber_filter.h>
# include <message_filters/subscriber.h>
# include <message_filters/sync_policies/exact_time.h>
# include <message_filters/synchronizer.h>

# include <sensor_msgs/Image.h>
# include <stereo_msgs/DisparityImage.h>


# include "hueblob/AddObject.h"
# include "hueblob/Blob.h"
# include "hueblob/Blobs.h"
# include "hueblob/TrackObject.h"



class CvHistogram;

/// \brief Define an object in the object database.
struct Object {
  double anchor_x;
  double anchor_y;
  double anchor_z;

  int nViews;
  CvHistogram** modelHistogram;
};

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
		     const sensor_msgs::ImageConstPtr& right,
		     const stereo_msgs::DisparityImageConstPtr& disparity_msg);

  /// \brief AddObject service callback.
  bool AddObjectCallback(hueblob::AddObject::Request& request,
			 hueblob::AddObject::Response& response);

  /// \brief TrackObject service callback.
  bool TrackObjectCallback(hueblob::TrackObject::Request& request,
			   hueblob::TrackObject::Response& response);

  /// \}

  /// \name Internal methods.
  /// \{

  void checkImagesSize(int height, int width);

  /// \}

 private:
  /// \brief Define the synchronization policy.
  typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::Image, sensor_msgs::Image, stereo_msgs::DisparityImage>
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

  /// \brief OpenCV bridge used for ROS<->OpenCv conversions.
  sensor_msgs::CvBridge bridge_;


  /// \brief Left image subscriber.
  image_transport::SubscriberFilter left_sub_;
  /// \brief Right image subscriber.
  image_transport::SubscriberFilter right_sub_;
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

  /// \brief AddObject service server.
  ros::ServiceServer AddObject_srv_;

  /// \brief TrackObject service server.
  ros::ServiceServer TrackObject_srv_;

  /// \brief Object database.
  ///
  /// This associates each object name to its definition.
  std::map<std::string, Object> objects_;

  // FIXME:
  cv::Ptr<IplImage> trackImage;
  cv::Ptr<IplImage> hstrackImage[2];
  cv::Ptr<IplImage> trackBackProj;
  cv::Ptr<IplImage> thrBackProj;

  // HueBlobObj& getObject(const char *name);
  // void addObject(const char *name, double anchor_x,
  //                       double anchor_y, double anchor_z);
};

#endif //! HUEBLOB_HUEBLOB_H
