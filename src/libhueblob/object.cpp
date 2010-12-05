#include "libhueblob/object.hh"

HueSaturation::HueSaturation(const IplImage& image)
  : h(cvCreateImage(cvGetSize(&image), 8, 1)),
    s(cvCreateImage(cvGetSize(&image), 8, 1)),
    planes()
{
  cv::Ptr<IplImage> hsv = cvCreateImage(cvGetSize(&image), 8, 3);
  cvCvtColor(&image, hsv, CV_BGR2HSV);

  cvCvtPixToPlane(hsv, h, NULL, NULL, NULL);
  cvCvtPixToPlane(hsv, NULL, s, NULL, NULL);

  planes[0] = h;
  planes[1] = h;
}

Object::Object()
  :  anchor_x(),
     anchor_y(),
     anchor_z(),
     modelHistogram()
{}

cv::Ptr<IplImage>
Object::computeMask(const IplImage& model)
{
  cv::Ptr<IplImage> gmodel = cvCreateImage(cvGetSize(&model), 8, 1);
  cv::Ptr<IplImage> mask = cvCreateImage(cvGetSize(&model), 8, 1);
  cvCvtColor(&model, gmodel, CV_BGR2GRAY);
  cvThreshold(gmodel, mask, 5, 255, CV_THRESH_BINARY);
  return mask;
}

void
Object::addView(const IplImage& model)
{
  // Histogram parameters initialization.
  int hist_size[] = {h_bins, s_bins};
  //  0 (~0°red) to 180 (~360°red again)
  float hue_range[] = { 0, 250 };
  //  0 (black-gray-white) to 255 (pure spectrum color)
  float sat_range[] = { 0, 250 };
  //  combine the two previous information
  float* hist_ranges[] = { hue_range, sat_range };

  // Compute the mask.
  cv::Ptr<IplImage> mask = computeMask(model);

  // Separate hue and saturation channels.
  HueSaturation hs(model);

  // Create the histogram.
  cv::Ptr<CvHistogram> objHist =
    cvCreateHist(2, hist_size, CV_HIST_ARRAY, hist_ranges, 1);

  // Compute the histogram.
  cvCalcHist(hs.planes, objHist, 0, mask);

  // Normalize.
  float max = 0.;
  cvGetMinMaxHistValue(objHist, 0, &max, 0, 0);
  cvConvertScale(objHist->bins, objHist->bins, max ? 255. / max : 0., 0);

  this->modelHistogram.push_back(objHist);
}
