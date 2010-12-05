#include <boost/foreach.hpp>
#include <boost/scope_exit.hpp>

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <hueblob/Blob.h>
#include <hueblob/AddObject.h>
#include <cv.h>

#include "libhueblob/hueblob.h"

void nullDeleter(void*) {}

HueBlob::HueBlob()
  : nh_("hueblob"),
    it_(nh_),
    stereo_topic_prefix_ (),
    threshold_(),
    bridge_(),
    left_sub_(),
    right_sub_(),
    disparity_sub_(),
    sync_(3),
    blobs_pub_(nh_.advertise<hueblob::Blobs>("blobs", 5)),
    AddObject_srv_(nh_.advertiseService
		   ("add_object", &HueBlob::AddObjectCallback, this)),
    ListObject_srv_(nh_.advertiseService
		    ("list_object", &HueBlob::ListObjectCallback, this)),
    RmObject_srv_(nh_.advertiseService
		  ("rm_object", &HueBlob::RmObjectCallback, this)),
    TrackObject_srv_(nh_.advertiseService
		   ("track_object", &HueBlob::TrackObjectCallback, this)),
    objects_(),
    check_synced_timer_(),
    left_received_(),
    right_received_(),
    disp_received_(),
    all_received_(),
    lastImage(),
    trackImage(),
    hstrackImage(),
    trackBackProj(),
    thrBackProj(),
    blobTrackImage()
{
  // Parameter initialization.
  ros::param::param<std::string>("~stereo", stereo_topic_prefix_, "");
  ros::param::param<double>("threshold", threshold_, 75.);

  // Initialize the node subscribers, publishers and filters.
  setupInfrastructure(stereo_topic_prefix_);
}

HueBlob::~HueBlob()
{
  ROS_DEBUG("Destructing the node.");
}

void
HueBlob::spin()
{
  typedef std::pair<const std::string&, const Object&> iterator_t;

  ros::Rate loop_rate(10);

  ROS_DEBUG("Entering the node main loop.");
  while (ros::ok())
    {
      hueblob::Blobs blobs;

      //FIXME: currently iterate on all blobs.
      // Should we prune untrackable blobs?
      // Should we provide a way to disable tracking
      //  for some objects?
      BOOST_FOREACH(iterator_t it, objects_)
	{
	  hueblob::Blob blob = trackBlob(it.first);
	  blobs.blobs.push_back(blob);
	}
      blobs_pub_.publish(blobs);

      ros::spinOnce();
      loop_rate.sleep();
    }
}

namespace
{
  void increment(int* value)
  {
    ++(*value);
  }
} // end of anonymous namespace.

void
HueBlob::setupInfrastructure(const std::string& stereo_prefix)
{
  ROS_INFO("%s",stereo_prefix.c_str());
  stereo_topic_prefix_ = nh_.resolveName(stereo_prefix);
  ROS_INFO("%s",stereo_topic_prefix_.c_str());

  const std::string left_topic =
    ros::names::clean(stereo_topic_prefix_ + "/left/image_rect_color");
  const std::string right_topic =
    ros::names::clean(stereo_topic_prefix_ + "/right/image_rect_color");
  const std::string disparity_topic =
    ros::names::clean(stereo_topic_prefix_ + "/disparity");

  left_sub_.subscribe(it_, left_topic, 3);
  right_sub_.subscribe(it_, right_topic, 3);
  disparity_sub_.subscribe(nh_, disparity_topic, 3);

  //FIXME: is it needed to be reentrant?
  //sync_.disconnectAll();
  sync_.connectInput(left_sub_, right_sub_, disparity_sub_);
  sync_.registerCallback(boost::bind(&HueBlob::imageCallback,
				     this, _1, _2, _3));

  // Complain every 30s if the topics appear unsynchronized
  left_sub_.registerCallback(boost::bind(increment, &left_received_));
  right_sub_.registerCallback(boost::bind(increment, &right_received_));
  disparity_sub_.registerCallback(boost::bind(increment, &disp_received_));
  sync_.registerCallback(boost::bind(increment, &all_received_));
  check_synced_timer_ =
    nh_.createWallTimer(ros::WallDuration(30.0),
			boost::bind(&HueBlob::checkInputsSynchronized, this));

  ROS_INFO("Subscribing to:\n\t* %s\n\t* %s\n\t* %s",
	   left_topic.c_str(), right_topic.c_str(),
	   disparity_topic.c_str());
}

void
HueBlob::imageCallback(const sensor_msgs::ImageConstPtr& left,
		       const sensor_msgs::ImageConstPtr& right,
		       const stereo_msgs::DisparityImageConstPtr& disparity_msg)
{
  lastImage = bridge_.imgMsgToCv(left,"bgr8");
}

bool
HueBlob::AddObjectCallback(hueblob::AddObject::Request& request,
			   hueblob::AddObject::Response& response)
{
  response.status = 0;
  IplImage* model;

  // Convert ROS image to OpenCV.
  try
    {
      boost::shared_ptr<sensor_msgs::Image> image_ptr
	(&request.image, nullDeleter);
      model = bridge_.imgMsgToCv(image_ptr,"rgb8");
    }
  catch(const sensor_msgs::CvBridgeException& error)
    {
      ROS_ERROR("failed to convert image");
      return false;
    }

  // Get reference on the object.
  Object& object = objects_[request.name];

  // Emit a warning if the object already exists.
  if (object.anchor_x
      || object.anchor_y
      || object.anchor_z)
    ROS_WARN("Overwriting the object %s", request.name.c_str());

  // Initialize the object.
  object.anchor_x = request.anchor.x;
  object.anchor_y = request.anchor.y;
  object.anchor_z = request.anchor.z;

  /// Add the view to the object.
  object.addView(*model);

  return true;
}

bool
HueBlob::ListObjectCallback(hueblob::ListObject::Request& request,
			    hueblob::ListObject::Response& response)
{
  typedef std::pair<const std::string&, const Object&> iterator_t;
  BOOST_FOREACH(iterator_t it, objects_)
    response.objects.push_back(it.first);
  return true;
}

bool
HueBlob::RmObjectCallback(hueblob::RmObject::Request& request,
			  hueblob::RmObject::Response& response)
{
  objects_.erase(request.name);
  return true;
}

bool
HueBlob::TrackObjectCallback(hueblob::TrackObject::Request& request,
			     hueblob::TrackObject::Response& response)
{
  return true;
}

namespace
{
  /* reset search zone if it is incorrect */
  void resetSearchZone(hueblob::Box& blob, const cv::Ptr<IplImage>& thrBackProj)
  {
    if (blob.x < 0 || blob.y < 0)
      {
	blob.x = 0;
	blob.y = 0;
	blob.width = thrBackProj->width-1;
	blob.height = thrBackProj->height-1;
      }

    if (blob.x + blob.width > thrBackProj->width - 1)
      blob.width = thrBackProj->width - 1 - blob.x;
    if (blob.y + blob.height > thrBackProj->height - 1)
      blob.height = thrBackProj->height - 1 - blob.y;
  }
} // end of anonymous namespace.


#define RESIZE_IF_NEEDED(IMG, COMPONENTS)                               \
  do {                                                                  \
    if (!IMG                                                            \
        || IMG->width != lastImage->width                               \
        || IMG->height != lastImage->height)                            \
      IMG = cvCreateImage(cvGetSize(lastImage), 8, COMPONENTS);         \
  } while(0)

hueblob::Blob
HueBlob::trackBlob(const std::string& name)
{
  hueblob::Box blob;
  hueblob::Blob blob_;
  blob_.name = name;

  // Image acquisition.
  if (!lastImage || !lastImage->width || !lastImage->height)
    return blob_;

  RESIZE_IF_NEEDED(blobTrackImage[0], 3);
  RESIZE_IF_NEEDED(blobTrackImage[1], 1);
  RESIZE_IF_NEEDED(blobTrackImage[2], 1);
  RESIZE_IF_NEEDED(thrBackProj, 3);
  RESIZE_IF_NEEDED(hstrackImage[0], 1);
  RESIZE_IF_NEEDED(hstrackImage[1], 1);

  cvCvtColor(lastImage, blobTrackImage[0], CV_BGR2HSV);

  for(unsigned i=1; i < 3; ++i)
    cvCvtColor(lastImage, blobTrackImage[i], CV_BGR2GRAY);

  // Blob detection.
  Object& object = objects_[name];
  if (object.modelHistogram.empty())
    {
      ROS_ERROR("Fail to track object as no view is available.");
      return blob_;
    }

  resetSearchZone(blob, thrBackProj);

  // convert source to HSV
  cvCvtPixToPlane(blobTrackImage[0], hstrackImage[0], NULL, NULL, NULL);
  cvCvtPixToPlane(blobTrackImage[0], NULL, hstrackImage[1], NULL, NULL);

  // iterate over all templates
  IplImage* hstrackImage_[2] = { hstrackImage[0], hstrackImage[1] };

  int nViews = object.modelHistogram.size();
  if (nViews <= 1)
    cvCalcBackProject(hstrackImage_, thrBackProj, object.modelHistogram[0]);
  else
    {
      memset(thrBackProj->imageData, 0, thrBackProj->imageSize);
      for(int nmodel = 0; nmodel < nViews; ++nmodel)
	{
	  cvCalcBackProject(hstrackImage_, trackBackProj,
			    object.modelHistogram[nmodel]);

	  unsigned char* s = (unsigned char *)trackBackProj->imageData;
	  unsigned char* d = (unsigned char *)thrBackProj->imageData;
	  for(int i = 0; i < thrBackProj->imageSize; ++i)
	    {
	      int p = *s + *d;
	      if (p > 255)
		p = 255;
	      *d = p;
	      s++; d++;
	    }
	}

      cvThreshold(thrBackProj, trackBackProj, 32, 0, CV_THRESH_TOZERO);
      cvSmooth(trackBackProj, thrBackProj, CV_MEDIAN, 3);

      CvBox2D box;
      CvConnectedComp components;
      CvTermCriteria criteria =
	cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 50, 1);
      cvCamShift(thrBackProj,
		 cvRect(blob.x, blob.y, blob.width, blob.height),
		 criteria, &components, &box);

      if (!box.size.height || !box.size.width)
	{
	  blob.x = blob.y = -1;
	  ROS_WARN("cannot find blob");
          return blob_;
	}

      blob.x = components.rect.x;
      blob.y = components.rect.y;
      blob.width = components.rect.width;
      blob.height = components.rect.height;
    }


  blob_.position.transform.translation.x = blob.x;
  blob_.position.transform.translation.y = blob.y;
  blob_.position.transform.translation.z = blob.width;
  return blob_;

  //FIXME: get depth information from disparity.
  //FIXME: compute average.
}

void
HueBlob::checkInputsSynchronized()
{
  int threshold = 3 * all_received_;
  if (left_received_ > threshold || right_received_ > threshold || disp_received_ > threshold)
    {
      ROS_WARN("[hueblob] Low number of synchronized left/right/disparity triplets received.\n"
	       "Left images received: %d\n"
	       "Right images received: %d\n"
	       "Disparity images received: %d\n"
	       "Synchronized triplets: %d\n"
	       "Possible issues:\n"
	       "\t* stereo_image_proc is not running.\n"
	       "\t* The cameras are not synchronized.\n"
	       "\t* The network is too slow. One or more images are dropped from each triplet.",
	       left_received_, right_received_, disp_received_, all_received_);
    }
}
