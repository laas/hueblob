#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

#include "hueblob/Blob.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hueblob");

  ros::NodeHandle n("hueblob");
  image_transport::ImageTransport it(n);

  ros::Publisher blob_pub = n.advertise<hueblob::Blob>("blob", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
    {
      hueblob::Blob blob;
      blob_pub.publish(blob);

      ros::spinOnce();
      loop_rate.sleep();
    }
}
