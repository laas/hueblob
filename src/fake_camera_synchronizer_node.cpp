#include <boost/bind.hpp>

#include <ros/console.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<
  sensor_msgs::Image, sensor_msgs::CameraInfo,
  sensor_msgs::Image, sensor_msgs::CameraInfo>
policy_t;

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_camera_synchronizer");

  std::string in, out;
  ros::param::param<std::string>("~in", in, "");
  ros::param::param<std::string>("~out", out, "");

  std::string left_in = in + "/left/image_raw";
  std::string left_cam_in = in + "/left/camera_info";
  std::string left_out = out + "/left/image_raw";
  std::string left_cam_out = out + "/left/camera_info";

  std::string right_in = in + "/right/image_raw";
  std::string right_cam_in = in + "/right/camera_info";
  std::string right_out = out + "/right/image_raw";
  std::string right_cam_out = out + "/right/camera_info";

  ros::NodeHandle n("fake_camera_synchronizer");
  image_transport::ImageTransport it(n);

  image_transport::Publisher pub_l = it.advertise(left_out, 1);
  image_transport::Publisher pub_r = it.advertise(right_out, 1);
  ros::Publisher pub_cam_l =
    n.advertise<sensor_msgs::CameraInfo>(left_cam_out, 1);
  ros::Publisher pub_cam_r =
    n.advertise<sensor_msgs::CameraInfo>(right_cam_out, 1);

  message_filters::Subscriber<sensor_msgs::Image>
    sub_l(n, left_in, 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo>
    sub_l_cam(n, left_cam_in, 1);
  message_filters::Subscriber<sensor_msgs::Image>
    sub_r(n, right_in, 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo>
    sub_r_cam(n, right_cam_in, 1);


  message_filters::Synchronizer<policy_t> sync
    (policy_t(10), sub_l, sub_l_cam, sub_r, sub_r_cam);
  sync.registerCallback
    (boost::bind(imageCallback,
		 boost::ref(pub_l), boost::ref(pub_cam_l),
		 boost::ref(pub_r), boost::ref(pub_cam_r),
		 _1, _2, _3, _4));

  ros::Rate loop_rate(10);

  while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
}
