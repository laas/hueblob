/* @(#)hueblob.h
 */

#ifndef _HUEBLOB_H
#define _HUEBLOB_H 1

#include "ros/ros.h"
#include "hueblob/AddObject.h"
#include "hueblob/Blob.h"
#include <vector>
#include "cv_bridge/CvBridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/Image.h"
#include <string>

class CvHistogram;
struct HueBlobObj {
  const char *name;
  double anchor_x, anchor_y, anchor_z;
  int nViews;
  CvHistogram **modelHistogram;
};

class HueBlob
{
 public:
  HueBlob(ros::NodeHandle &n, std::string left_cam_topic, std::string right_cam_topic);
  virtual ~HueBlob(){};
  bool AddObjectServ(hueblob::AddObject::Request &req,
                 hueblob::AddObject::Response &res);
  hueblob::Blob GetBlobs();

 private:
  ros::NodeHandle n_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  sensor_msgs::CvBridge bridge_;
  image_transport::Publisher image_pub_;
  std::string left_cam_topic_;
  std::string right_cam_topic_;

  std::vector<HueBlobObj*> objects_;
  HueBlobObj* GetObject(const char *name);
  HueBlobObj* AddObject(const char *name, double anchor_x,
                        double anchor_y, double anchor_z);
};

#endif /* _HUEBLOB_H */

