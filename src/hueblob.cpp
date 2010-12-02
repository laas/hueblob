#include "hueblob.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <ros/param.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <hueblob/Blob.h>
#include <hueblob/AddObject.h>
#include <cv.h>


using namespace std;
void nullDeleter(void*) {}

HueBlob::HueBlob(ros::NodeHandle &n,
                 string left_cam_topic, string right_cam_topic):
  n_(n), it_(n_), left_cam_topic_(left_cam_topic), right_cam_topic_(right_cam_topic)
{
}

hueblob::Blob HueBlob::GetBlobs(){
  return hueblob::Blob();
}


bool HueBlob::AddObjectServ(hueblob::AddObject::Request &req,
                hueblob::AddObject::Response &res)
{
  CvHistogram **objHist;
  IplImage *model, *gmodel, *mask;
  IplImage *hsv, *hs_planes[2];
  float max;
  int hist_size[] = {25, 25};
  float hue_range[] = { 0, 250 }; /* 0 (~0°red) to 180 (~360°red again) */
  float sat_range[] = { 0, 250 }; /* 0 (black-gray-white) to 255 (pure spectrum color) */
  float *hist_ranges[] = { hue_range, sat_range };
  boost::shared_ptr<sensor_msgs::Image> image_ptr(&req.image, nullDeleter);

  model = bridge_.imgMsgToCv(image_ptr,"rgb8");
  /* create object structure */
  HueBlobObj *object = GetObject(req.name.c_str());
  res.status = 0;
  if (!object) {
    object = AddObject(req.name.c_str(),
		       req.anchor_x, req.anchor_y, req.anchor_z);
    if (!object) {
      res.status = 1;
      return false;
    }
  }

  object->nViews++;
  object->modelHistogram =
    (CvHistogram **)realloc(object->modelHistogram,
			    object->nViews * sizeof(*object->modelHistogram));
  objHist = &object->modelHistogram[object->nViews-1];

  /* compute mask */
  gmodel = cvCreateImage(cvGetSize(model), 8, 1);
  mask = cvCreateImage(cvGetSize(model), 8, 1);
  cvCvtColor(model, gmodel, CV_BGR2GRAY);
  cvThreshold(gmodel, mask, 5, 255, CV_THRESH_BINARY);

  /* create histogram */
  hsv = cvCreateImage(cvGetSize(model), 8, 3);
  hs_planes[0] = cvCreateImage(cvGetSize(model), 8, 1);
  hs_planes[1] = cvCreateImage(cvGetSize(model), 8, 1);

  cvCvtColor(model, hsv, CV_BGR2HSV);
  cvCvtPixToPlane(hsv, hs_planes[0], NULL, NULL, NULL);
  cvCvtPixToPlane(hsv, NULL, hs_planes[1], NULL, NULL);

  *objHist = cvCreateHist(2, hist_size, CV_HIST_ARRAY, hist_ranges, 1);

  /* compute histogram */
  cvCalcHist(hs_planes, *objHist, 0, mask);
  cvGetMinMaxHistValue(*objHist, 0, &max, 0, 0 );
  cvConvertScale((*objHist)->bins, (*objHist)->bins, max?255./max:0., 0);

  cvReleaseImage(&hs_planes[1]);
  cvReleaseImage(&hs_planes[0]);
  cvReleaseImage(&hsv);
  cvReleaseImage(&mask);
  cvReleaseImage(&gmodel);
  cvReleaseImage(&model);
  return true;
}

HueBlobObj *
HueBlob::GetObject(const char *name)
{
  for (  std::vector<HueBlobObj*>::iterator iter= objects_.begin();
         iter != objects_.end(); iter++ )
    {
      if (!strcmp((*iter)->name, name)) return (*iter);
    }
  return NULL;
}


HueBlobObj *
HueBlob::AddObject(const char *name, double anchor_x,
                   double anchor_y, double anchor_z)
{
  HueBlobObj* new_obj = new HueBlobObj();

  new_obj->name = strdup(name);
  new_obj->anchor_x = anchor_x;
  new_obj->anchor_y = anchor_y;
  new_obj->anchor_z = anchor_z;
  new_obj->nViews = 0;
  new_obj->modelHistogram = NULL;
  objects_.push_back(new_obj);
    return new_obj;
}
