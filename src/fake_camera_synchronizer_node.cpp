#include <boost/bind.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

#include "hueblob/Blob.h"

sensor_msgs::ImageConstPtr left;
sensor_msgs::ImageConstPtr right;

void imageCallback(sensor_msgs::ImageConstPtr& image,
		   image_transport::Publisher& pub_l,
		   image_transport::Publisher& pub_r,
		   const sensor_msgs::ImageConstPtr& msg)
{
  image = msg;

  if (left && right)
    {
      sensor_msgs::Image image_left(*left);
      sensor_msgs::Image image_right(*right);

      pub_l.publish(image_left);
      pub_r.publish(image_right);

      left.reset();
      right.reset();
    }
}

image_transport::CameraSubscriber::Callback
bindImageCallback(sensor_msgs::ImageConstPtr& image,
		  image_transport::Publisher& pub_l,
		  image_transport::Publisher& pub_r)
{
  return boost::bind
    (imageCallback,
     boost::ref(image), boost::ref(pub_l), boost::ref(pub_r), _1);
}


int main(int argc, char **argv)
{
  std::string left_in, left_out, right_in, right_out;
  ros::param::param<std::string>("~left_in", left_in, "");
  ros::param::param<std::string>("~right_in", right_in, "");
  ros::param::param<std::string>("~left_out", left_out, "");
  ros::param::param<std::string>("~right_out", right_out, "");


  ros::init(argc, argv, "fake_camera_synchronizer");

  ros::NodeHandle n("fake_camera_synchronizer");
  image_transport::ImageTransport it(n);

  image_transport::Publisher pub_l = it.advertise(left_out, 1);
  image_transport::Publisher pub_r = it.advertise(right_out, 1);

  image_transport::CameraSubscriber sub_l =
    it.subscribeCamera(left_in, 1, bindImageCallback(left, pub_l, pub_r));
  image_transport::CameraSubscriber sub_r =
    it.subscribeCamera(right_in, 1, bindImageCallback(right, pub_l, pub_r));

  ros::Rate loop_rate(10);

  while (ros::ok())
    {
      ros::spinOnce();
      loop_rate.sleep();
    }
}
