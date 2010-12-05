#include "libhueblob/object.hh"

Object::Object()
  :  anchor_x(),
     anchor_y(),
     anchor_z(),
     modelHistogram()
{}

void
Object::addView(const IplImage& model)
{
  int hist_size[] = {h_bins, s_bins};
  // 0 (~0°red) to 180 (~360°red again)
  float hue_range[] = { 0, 250 };
  // 0 (black-gray-white) to 255 (pure spectrum color)
  float sat_range[] = { 0, 250 };
  float* hist_ranges[] = { hue_range, sat_range };

  // compute mask
  cv::Ptr<IplImage> gmodel = cvCreateImage(cvGetSize(&model), 8, 1);
  cv::Ptr<IplImage> mask = cvCreateImage(cvGetSize(&model), 8, 1);
  cvCvtColor(&model, gmodel, CV_BGR2GRAY);
  cvThreshold(gmodel, mask, 5, 255, CV_THRESH_BINARY);

  // create histogram
  cv::Ptr<IplImage> hsv = cvCreateImage(cvGetSize(&model), 8, 3);
  cv::Ptr<IplImage> hs_planes[2];
  hs_planes[0] = cvCreateImage(cvGetSize(&model), 8, 1);
  hs_planes[1] = cvCreateImage(cvGetSize(&model), 8, 1);

  cvCvtColor(&model, hsv, CV_BGR2HSV);
  cvCvtPixToPlane(hsv, hs_planes[0], NULL, NULL, NULL);
  cvCvtPixToPlane(hsv, NULL, hs_planes[1], NULL, NULL);

  cv::Ptr<CvHistogram> objHist =
    cvCreateHist(2, hist_size, CV_HIST_ARRAY, hist_ranges, 1);

  // compute histogram
  float max = 0.;
  IplImage* hs_planes_[] = {hs_planes[0], hs_planes[1]};
  cvCalcHist(hs_planes_, objHist, 0, mask);
  cvGetMinMaxHistValue(objHist, 0, &max, 0, 0 );
  cvConvertScale(objHist->bins, objHist->bins, max ? 255. / max : 0., 0);

  this->modelHistogram.push_back(objHist);
}
