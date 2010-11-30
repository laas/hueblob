#include <boost/bind.hpp>

#include <ros/console.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

#include "hueblob/Blob.h"

sensor_msgs::ImageConstPtr left;
sensor_msgs::ImageConstPtr right;

void publishImagesIfReady(image_transport::Publisher& pub_l,
			  image_transport::Publisher& pub_r)
{
  if (!left || !right)
    return;
  sensor_msgs::Image image_left(*left);
  sensor_msgs::Image image_right(*right);

  // Fix timestamps if necessary.

  // FIXME: manual synchronization should be reported and
  // a warning should be issued if the difference between
  // timestamps is too important.
  if (image_left.header.stamp != image_right.header.stamp)
    {
      if (image_left.header.stamp >= image_right.header.stamp)
	image_right.header.stamp = image_left.header.stamp;
      else
	image_left.header.stamp = image_right.header.stamp;
    }

  pub_l.publish(image_left);
  pub_r.publish(image_right);

  left.reset();
  right.reset();
}

void imageCallback(sensor_msgs::ImageConstPtr& image,
		   image_transport::Publisher& pub_l,
		   image_transport::Publisher& pub_r,
		   const sensor_msgs::ImageConstPtr& msg)
{
  image = msg;
  publishImagesIfReady(pub_l, pub_r);
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
  ros::init(argc, argv, "fake_camera_synchronizer");

  std::string left_in, left_out, right_in, right_out;
  ros::param::param<std::string>("~left_in", left_in, "");
  ros::param::param<std::string>("~right_in", right_in, "");
  ros::param::param<std::string>("~left_out", left_out, "");
  ros::param::param<std::string>("~right_out", right_out, "");


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
