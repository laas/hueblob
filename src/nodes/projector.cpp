#include <ros/ros.h>
#include <nodelet/loader.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "projector", ros::init_options::AnonymousName);
  // if (ros::names::remap("image") == "image") {
  //   ROS_WARN("Topic 'image' has not been remapped! Typical command-line usage:\n"
  //            "\t$ rosrun image_view image_view image:=<image topic> [transport]");
  // }

  nodelet::Loader manager(false);
  nodelet::M_string remappings;
  nodelet::V_string my_argv(argv , argv + argc);

  ROS_INFO("Loading nodelet");
  manager.load(ros::this_node::getName(), "hueblob/projector", remappings, my_argv);

  ros::spin();
  return 0;
}
