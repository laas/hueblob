#include <ros/ros.h>
#include "libhueblob/hueblob.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hueblob");

  // Instantiate the main object.
  HueBlob hueblob;

  // Launch main loop (will not return until the program terminates).
  hueblob.spin();
}
