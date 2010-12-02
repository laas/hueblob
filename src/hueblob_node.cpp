#include "hueblob.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include "hueblob/Blob.h"
#include "hueblob/AddObject.h"
#include "hueblob/TwoInts.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hueblobserv");

  ros::NodeHandle n("hueblob");
  image_transport::ImageTransport it(n);
  ros::Publisher blob_pub = n.advertise<hueblob::Blob>("blob", 1000);
  ros::Rate loop_rate(10);
  HueBlob h = HueBlob(n,"left_cam","right_cam");
  ros::ServiceServer blob_service = n.advertiseService("add_object",
                                                       &HueBlob::AddObjectServ, &h);

  while (ros::ok())
    {
      hueblob::Blob blob = h.GetBlobs();
      blob_pub.publish(blob);

      ros::spinOnce();
      loop_rate.sleep();
    }
}
