#include <boost/foreach.hpp>
#include <boost/scope_exit.hpp>

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <hueblob/Blob.h>
#include <hueblob/AddObject.h>
#include <cv.h>

#include "hueblob.h"

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
		   ("add_objet", &HueBlob::AddObjectCallback, this)),
    TrackObject_srv_(nh_.advertiseService
		   ("track_objet", &HueBlob::TrackObjectCallback, this)),
    objects_()
{
  // Parameter initialization.
  ros::param::param<std::string>("stereo", stereo_topic_prefix_, "");
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
  ros::Rate loop_rate(10);

  ROS_DEBUG("Entering the node main loop.");
  while (ros::ok())
    {
      trackBlob();

      hueblob::Blobs blobs;
      //FIXME: fill structure before publishing.
      blobs_pub_.publish(blobs);

      ros::spinOnce();
      loop_rate.sleep();
    }
}

void
HueBlob::setupInfrastructure(const std::string& stereo_prefix)
{
  stereo_topic_prefix_ = nh_.resolveName(stereo_prefix);

  const std::string left_topic =
    ros::names::clean(stereo_prefix + "/left/image_mono");
  const std::string right_topic =
    ros::names::clean(stereo_prefix + "/right/image_mono");
  const std::string disparity_topic =
    ros::names::clean(stereo_prefix + "/disparity");

  left_sub_.subscribe(it_, left_topic, 3);
  right_sub_.subscribe(it_, right_topic, 3);
  disparity_sub_.subscribe(nh_, disparity_topic, 3);

  //FIXME: is it needed to be reentrant?
  //sync_.disconnectAll();
  sync_.connectInput(left_sub_, right_sub_, disparity_sub_);
  sync_.registerCallback(boost::bind(&HueBlob::imageCallback,
				     this, _1, _2, _3));

  //FIXME: add callback checking that images are received.

  ROS_INFO("Subscribing to:\n\t* %s\n\t* %s\n\t* %s",
	   left_topic.c_str(), right_topic.c_str(),
	   disparity_topic.c_str());
}

void
HueBlob::imageCallback(const sensor_msgs::ImageConstPtr& left,
		       const sensor_msgs::ImageConstPtr& right,
		       const stereo_msgs::DisparityImageConstPtr& disparity_msg)
{
  lastImage = bridge_.imgMsgToCv(left,"rgb8");
}

bool
HueBlob::AddObjectCallback(hueblob::AddObject::Request& request,
			   hueblob::AddObject::Response& response)
{
  CvHistogram** objHist;
  cv::Ptr<IplImage> gmodel;
  cv::Ptr<IplImage> mask;
  cv::Ptr<IplImage> hsv;
  cv::Ptr<IplImage> hs_planes[2];
  float max;
  int hist_size[] = {25, 25};
  // 0 (~0°red) to 180 (~360°red again)
  float hue_range[] = { 0, 250 };
  // 0 (black-gray-white) to 255 (pure spectrum color)
  float sat_range[] = { 0, 250 };
  float* hist_ranges[] = { hue_range, sat_range };
  cv::Ptr<IplImage> model;

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

  ++object.nViews;

  //FIXME: use a C++ container instead.
  object.modelHistogram =
    (CvHistogram **)realloc(object.modelHistogram,
			    object.nViews * sizeof(*object.modelHistogram));
  objHist = &object.modelHistogram[object.nViews-1];

  // compute mask
  gmodel = cvCreateImage(cvGetSize(model), 8, 1);
  mask = cvCreateImage(cvGetSize(model), 8, 1);
  cvCvtColor(model, gmodel, CV_BGR2GRAY);
  cvThreshold(gmodel, mask, 5, 255, CV_THRESH_BINARY);

  // create histogram
  hsv = cvCreateImage(cvGetSize(model), 8, 3);
  hs_planes[0] = cvCreateImage(cvGetSize(model), 8, 1);
  hs_planes[1] = cvCreateImage(cvGetSize(model), 8, 1);

  cvCvtColor(model, hsv, CV_BGR2HSV);
  cvCvtPixToPlane(hsv, hs_planes[0], NULL, NULL, NULL);
  cvCvtPixToPlane(hsv, NULL, hs_planes[1], NULL, NULL);

  *objHist = cvCreateHist(2, hist_size, CV_HIST_ARRAY, hist_ranges, 1);

  // compute histogram
  IplImage* hs_planes_[] = {hs_planes[0], hs_planes[1]};
  cvCalcHist(hs_planes_, *objHist, 0, mask);
  cvGetMinMaxHistValue(*objHist, 0, &max, 0, 0 );
  cvConvertScale((*objHist)->bins, (*objHist)->bins, max?255./max:0., 0);

  return true;
}

bool
HueBlob::TrackObjectCallback(hueblob::TrackObject::Request& request,
			     hueblob::TrackObject::Response& response)
{
  return true;
}

void
HueBlob::trackBlob()
{
  Object object;

  // Image.
#ifdef FROM_GENOM
  struct HueBlobObj *objmodel;
  CvTermCriteria criteria = cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 50, 1);
  CvConnectedComp components;
  CvBox2D object;
  unsigned char *s, *d;
  unsigned int p;
  int nmodel;
  double intrinsic[9], baseline;
  IplImage viam;
  T3D pose;
  int i;
  DATA_IM3D im3d;
  T3D c2w;

  /* --- retrieve video image --- */

  if (posterTake(blobImagePosterId, POSTER_READ) != OK) {
    *report = errnoGet();
    return ETHER;
  }
  baseline = blobImage->calibration.baseline;
  memcpy(intrinsic, blobImage->image[0].calibration.intrirect,
	 sizeof(intrinsic));
  c2w = t3dBryanIdentity;
  c2w.bryan.yaw = blobImage->image[0].pos.sensorToMain.euler.yaw;
  c2w.bryan.pitch = blobImage->image[0].pos.sensorToMain.euler.pitch;
  c2w.bryan.roll = blobImage->image[0].pos.sensorToMain.euler.roll;
  c2w.bryan.x = blobImage->image[0].pos.sensorToMain.euler.x;
  c2w.bryan.y = blobImage->image[0].pos.sensorToMain.euler.y;
  c2w.bryan.z = blobImage->image[0].pos.sensorToMain.euler.z;
  c2w.flags |= T3D_ALLOW_CONVERSION;
  for(i=0; i<2; i++) {
    cvInitImageHeader(&viam,
		      cvSize(blobImage->image[i].width,
			     blobImage->image[i].height),
		      blobImage->image[i].depth,
		      blobImage->image[i].nChannels,
		      IPL_ORIGIN_TL, 4);
    cvSetData(&viam,
	      &blobImage->image[i].data[blobImage->image[i].dataOffset],
	      blobImage->image[i].widthStep);
    cvCvtColor(&viam, blobtrackImage[i+1], CV_BGR2GRAY);
    if (i == 0)
      cvCvtColor(&viam, blobtrackImage[0], CV_BGR2HSV);
  }
  posterGive(blobImagePosterId);

  /* --- detect blob --- */

  objmodel = getObject(blob->object);
  if (!objmodel || objmodel->nViews == 0) {
    *report = S_hueblob_NO_MODEL;
    return ETHER;
  }

  /* reset search zone if it is incorrect */
  if (blob->x < 0 || blob->y < 0) {
    blob->x = 0;
    blob->y = 0;
    blob->width = thrBackProj->width-1;
    blob->height = thrBackProj->height-1;
  }
  if (blob->x+blob->width > thrBackProj->width-1)
    blob->width = thrBackProj->width-1 - blob->x;
  if (blob->y+blob->height > thrBackProj->height-1)
    blob->height = thrBackProj->height-1 - blob->y;

  /* convert source to HSV */
  cvCvtPixToPlane(blobtrackImage[0], hstrackImage[0], NULL, NULL, NULL);
  cvCvtPixToPlane(blobtrackImage[0], NULL, hstrackImage[1], NULL, NULL);

  /* iterate over all templates */
  if (objmodel->nViews <= 1) {

    cvCalcBackProject(hstrackImage, thrBackProj, objmodel->modelHistogram[0]);

  } else {

    memset(thrBackProj->imageData, 0, thrBackProj->imageSize);
    for(nmodel = 0; nmodel<objmodel->nViews; nmodel++) {
      cvCalcBackProject(hstrackImage, trackBackProj, objmodel->modelHistogram[nmodel]);

      s = (unsigned char *)trackBackProj->imageData;
      d = (unsigned char *)thrBackProj->imageData;
      for(i=0;i<thrBackProj->imageSize;i++) {
	p = *s + *d; if (p>255) p = 255;
	*d = p;

	s++; d++;
      }
    }
  }

  cvThreshold(thrBackProj, trackBackProj, 32, 0, CV_THRESH_TOZERO);
  cvSmooth(trackBackProj, thrBackProj, CV_MEDIAN, 3);

  i = cvCamShift(thrBackProj,
		 cvRect(blob->x, blob->y, blob->width, blob->height),
		 criteria, &components, &object);
  if (object.size.height == 0 || object.size.width == 0) {
    blob->x = blob->y = -1;
    warnx("cannot find blob");
#ifdef HUEBLOB_DISPLAY
    if (blob->feedback == HUEBLOB_ENABLE) {
	cvNamedWindow("Blob image", 0);
	cvResizeWindow("Blob image", 320, 240);
	cvShowImage("Blob image", blobtrackImage[0]);
    }
#endif
    return EXEC;
  }

  blob->x = components.rect.x;
  blob->y = components.rect.y;
  blob->width = components.rect.width;
  blob->height = components.rect.height;

#ifdef HUEBLOB_DISPLAY
  if (blob->feedback == HUEBLOB_ENABLE) {
    cvRectangle(blobtrackImage[0],
		cvPoint(blob->x, blob->y),
		cvPoint(blob->x+blob->width, blob->y+blob->height),
		cvScalar(255), 1);
    cvNamedWindow("Blob image", 0);
    cvResizeWindow("Blob image", 320, 240);
    cvShowImage("Blob image", blobtrackImage[0]);
  }
#endif /* HUEBLOB_DISPLAY */

  /* --- compute 3D position --- */

  {
    int off, hei;

    off = blob->y - 4; if (off < 0) off = 0;
    hei = blob->height + 4; if (off+hei > blobtrackImage[1]->height)
			      hei = blobtrackImage[1]->height - off;

  pose = t3dMatrixIdentity;

  if (init_data_im3d(&im3d,
		     hei, blobtrackImage[1]->width, 0,0,0) != ERR_IM3D_OK) {
    return EXEC;
  }

  {
    unsigned char *images[2];

    intrinsic[5] -= off;

    images[0] = (unsigned char*)blobtrackImage[1]->imageData + off*blobtrackImage[1]->widthStep;
    images[1] = (unsigned char*)blobtrackImage[2]->imageData + off*blobtrackImage[2]->widthStep;

    if (hb_correlation_compute(images, blobtrackImage[1]->width, hei,
			       baseline, intrinsic, &im3d, report) != OK) {
      posterTake(blobCoordPosterId, POSTER_WRITE);
      memset(blobCoord, 0, sizeof(*blobCoord));
      posterGive(blobCoordPosterId);
      empty_data_im3d(&im3d);
      return EXEC;
    }
  }

  {
    DATA_PT3D *b;
    unsigned char *p;
    double x, y, z;
    int i, j, n, t;

    /* compute average */
    x = y = z = 0;
    n = t = 0;
    b = im3d.points_tab + blob->x;
    p = (unsigned char *)thrBackProj->imageData +
      off * thrBackProj->widthStep + blob->x;
    for(i = 0; i < blob->height; i++) {
      for(j = 0; j < blob->width; j++) {
	if (b->state == GOOD_PT3D) {
	  x += b->coord_1 * (*p);
	  y += b->coord_2 * (*p);
	  z += b->coord_3 * (*p);
	  n += *p;
	  t++;
	}
	b++;
	p++;
      }
      b += im3d.header.nbcol - blob->width;
      p += thrBackProj->widthStep - blob->width;
    }

    fprintf(stderr, "using %d 3d points\n", t);
    if (n <= 0) {
      posterTake(blobCoordPosterId, POSTER_WRITE);
      memset(blobCoord, 0, sizeof(*blobCoord));
      posterGive(blobCoordPosterId);
      empty_data_im3d(&im3d);
      return EXEC;
    }

    /* XXX temporarily convert camera 3d frame to pom's one */
    pose.matrix.matrix[3] = z / n + 0.02;
    pose.matrix.matrix[7] = -x / n;
    pose.matrix.matrix[11] = -y / n;
  }}

  empty_data_im3d(&im3d);

  t3dComp(&pose, &c2w);
  posterTake(blobCoordPosterId, POSTER_WRITE);
  *blobCoord = pose;
  posterGive(blobCoordPosterId);

  if (blob->feedback == HUEBLOB_ENABLE)
    t3dFPrint(stdout, &pose);

  return EXEC;
#endif
}
