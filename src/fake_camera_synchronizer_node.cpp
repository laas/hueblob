#include <boost/bind.hpp>

#include <ros/console.h>
#include <ros/param.h>
#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

// This node takes care of resynchronizing images and camera
// information to enforce time stamp matching.
// The main interest of this node is being able to realize stereo
// image processing when left and right camera image acquisition
// are not synchronized. This is required for Firewire cameras
// when using camera1394 (in CBoxTurtle release) or usb_cam.
//
// Here is how the can be used, it makes the assumption that a
// stereo camera pair topics are setup in the /stereo prefix and
// that you want to get a synchronized stereo pair in /stereo_sync.
//
// Input:
// - /stereo/left/image_raw
// - /stereo/left/camera_info
// - /stereo/right/image_raw
// - /stereo/right/camera_info
//
// Output:
// - /stereo_sync/left/image_raw
// - /stereo_sync/left/camera_info
// - /stereo_sync/right/image_raw
// - /stereo_sync/right/camera_info
//
// Launch the following command to start the synchronization mode:
// rosrun hueblob fake_camera_synchronizer_node _in:=/stereo _out:=/stereo_sync
//
// Then the stereo image processing node can be started:
// ROS_NAMESPACE=/stereo_sync rosrun stereo_image_proc stereo_image_proc


/// \brief Define policy type for Synchronizer filter.
///
/// This will matches image and camera information for
/// both left and right cameras.
typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo>
policy_t;


/// \brief Main callback which takes care of republishing.
///
/// This callback is called by the Synchronizer filter when
/// messages for both left/right images and cameras information
/// have been collected.
///
/// The callback the left image timestamps in all the other messages.
///
/// \param pub_l left image publisher (binded)
/// \param pub_cam_l left camera information publisher (binded)
/// \param pub_r right image publisher (binded)
/// \param pub_cam_r right camera information publisher (binded)
/// \param left left image
/// \param left_camera left camera information
/// \param right right image
/// \param right_camera right camera information
void imageCallback(image_transport::Publisher& pub_l,
		   ros::Publisher& pub_cam_l,
		   image_transport::Publisher& pub_r,
		   ros::Publisher& pub_cam_r,
		   const sensor_msgs::ImageConstPtr& left,
		   const sensor_msgs::CameraInfoConstPtr& left_camera,
		   const sensor_msgs::ImageConstPtr& right,
		   const sensor_msgs::CameraInfoConstPtr& right_camera
		   )
{
  sensor_msgs::Image left_(*left);
  sensor_msgs::Image right_(*right);
  sensor_msgs::CameraInfo left_camera_(*left_camera);
  sensor_msgs::CameraInfo right_camera_(*right_camera);

  // Fix timestamps.
  // FIXME: a warning should be issued if the differences between
  // timestamps are too important.
  left_camera_.header.stamp = left_.header.stamp;
  right_camera_.header.stamp = left_.header.stamp;
  right_.header.stamp = left_.header.stamp;

  pub_l.publish(left_);
  pub_cam_l.publish(left_camera_);
  pub_r.publish(right_);
  pub_cam_r.publish(right_camera_);
}


/// \brief Main entry point.
int main(int argc, char **argv)
{
  // ROS initialization.
  ros::init(argc, argv, "fake_camera_synchronizer");

  // Parameters definition.
  std::string in, out;
  ros::param::param<std::string>("~in", in, "");
  ros::param::param<std::string>("~out", out, "");

  // Topic name construction.
  std::string left_in = in + "/left/image_raw";
  std::string left_cam_in = in + "/left/camera_info";
  std::string left_out = out + "/left/image_raw";
  std::string left_cam_out = out + "/left/camera_info";

  std::string right_in = in + "/right/image_raw";
  std::string right_cam_in = in + "/right/camera_info";
  std::string right_out = out + "/right/image_raw";
  std::string right_cam_out = out + "/right/camera_info";

  // Node initialization.
  ros::NodeHandle n("fake_camera_synchronizer");
  image_transport::ImageTransport it(n);

  // Publishers creation.
  image_transport::Publisher pub_l = it.advertise(left_out, 1);
  image_transport::Publisher pub_r = it.advertise(right_out, 1);
  ros::Publisher pub_cam_l =
    n.advertise<sensor_msgs::CameraInfo>(left_cam_out, 1);
  ros::Publisher pub_cam_r =
    n.advertise<sensor_msgs::CameraInfo>(right_cam_out, 1);

  // Subscribers creation.
  message_filters::Subscriber<sensor_msgs::Image>
    sub_l(n, left_in, 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo>
    sub_l_cam(n, left_cam_in, 1);
  message_filters::Subscriber<sensor_msgs::Image>
    sub_r(n, right_in, 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo>
    sub_r_cam(n, right_cam_in, 1);

  // Message filter creation.
  message_filters::Synchronizer<policy_t> sync
    (policy_t(10), sub_l, sub_l_cam, sub_r, sub_r_cam);
  sync.registerCallback
    (boost::bind(imageCallback,
		 boost::ref(pub_l), boost::ref(pub_cam_l),
		 boost::ref(pub_r), boost::ref(pub_cam_r),
		 _1, _2, _3, _4));

  ros::spin();
}
