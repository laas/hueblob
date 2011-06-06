#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include "libhueblob/object.hh"
#include <algorithm>
#include <iostream>
#include "highgui.h"
// Histogram parameters initialization.
static const int hist_size[] = {Object::h_bins, Object::s_bins};
//  0 (~0°red) to 180 (~360°red again)
static const float hue_range[] = { 0, 250 };
//  0 (black-gray-white) to 255 (pure spectrum color)
static const float sat_range[] = { 0, 250 };
//  combine the two previous information
static const float* ranges[] = { hue_range, sat_range };


Object::Object()
  :
     algo(CAMSHIFT),
     anchor_x(),
     anchor_y(),
     anchor_z(),
     modelHistogram(),
     searchWindow_(-1, -1, -1, -1)
{}

cv::Mat
Object::computeMask(const cv::Mat& model)
{
  cv::Mat gmodel(model.size(), CV_8UC1);
  cv::Mat mask(model.size(), CV_8UC1);

  cv::cvtColor(model, gmodel, CV_BGR2GRAY);

  cv::threshold(gmodel, mask, 5, 255, CV_THRESH_BINARY);
  return mask;
}
void
Object::addView(const cv::Mat& model)
{
  // Compute the mask.
  cv::Mat mask = computeMask(model);
  // cv::namedWindow("test", CV_WINDOW_AUTOSIZE);
  // cv::imshow("test", model );
  // cv::waitKey();

  // Compute the histogram.
  //  only use channels 0 and 1 (hue and saturation).
  int channels[] = {0, 1};
  cv::Mat hsv;
  cv::MatND hist;
  cv::cvtColor(model, hsv, CV_BGR2HSV);
  // cv::imshow("test", hsv );
  // cv::waitKey();
  using namespace std;
  calcHist(&hsv, 1, channels, mask,
	   hist, 2, hist_size, ranges,
	   true, false);

  // Normalize.

  double max = 0.;
  cv::minMaxLoc(hist, 0, &max, 0, 0);

  //  convert MatND into Mat, no copy is done, two types will be
  //  merged soon enough.
  cv::Mat hist_(hist);
  cv::convertScaleAbs(hist_, hist_, max ? 255. / max : 0., 0);
  this->modelHistogram.push_back(hist);

  // compute histogram and thresholds for naive method
  getThresholds(hsv);
}

void
Object::getThresholds(const cv::Mat& hsv_img)
{
  cv::MatND hist;
  cv::Mat mask(hsv_img.size(), CV_8UC1);
  cv::inRange(hsv_img, cv::Scalar(0,50,50),
	      cv::Scalar(255, 255, 255),mask);
  int channels[] = {0};
  unsigned hbins = 255;
  const int hist_size[] = {hbins};
  float hranges[] = {0,255};
  const float* ranges[] = { hranges };
  cv::calcHist(&hsv_img, 1, channels, mask,
               hist, 1, hist_size,
               ranges, true, false);
  // normalize hist
  cv::normalize(hist, hist, 1, 0, cv::NORM_L1);
  double maxVal;
  int maxIdx;
  minMaxLoc(hist, NULL, &maxVal, NULL, &maxIdx);
  peak_color[0] = maxIdx;
  peak_color[1] = 200;
  peak_color[2] = 200;
  unsigned maxh(0), minh(255);
  for (unsigned h = 0; h < hbins; h++)
    {
      if (abs(float(h) - float(maxIdx)) > 20 || hist.at<float>(h) < 0.02*maxVal)
        continue;
      maxh = std::max(maxh, h);
      minh = std::min(minh, h);
    }

  cv::inRange(hsv_img, cv::Scalar(minh,50,50),
	      cv::Scalar(maxh, 255, 255),mask);


  cv::Mat hchan(hsv_img.size(), CV_8UC1);
  cv::Mat schan(hsv_img.size(), CV_8UC1);
  cv::Mat vchan(hsv_img.size(), CV_8UC1);
  cv::Mat chans[] = {hchan, schan, vchan};
  cv::split(hsv_img, chans);

  minMaxLoc(hchan, &lower_hue[0], &upper_hue[0], 0, 0, mask);
  minMaxLoc(schan, &lower_hue[1], &upper_hue[1], 0, 0, mask);
  minMaxLoc(vchan, &lower_hue[2], &upper_hue[2], 0, 0, mask);
  // Tolerance for s and v
  upper_hue[1] = 255;
  upper_hue[2] = 255;
  // float tol = 1.1;
  // lower[1]/= tol;o
  // lower[2]/= tol;
  // upper[1]*= tol;
  // upper[2]*= tol;
  std::cout << maxIdx << " " << std::endl;
  std::cout << lower_hue[0] << " "
            << lower_hue[1] << " "
            << lower_hue[2] << " " << std::endl;
  std::cout << upper_hue[0] << " "
            << upper_hue[1] << " "
            << upper_hue[2] << " " << std::endl;
}

namespace
{
  /// \brief Reset search zone if it is incorrect.
  void resetSearchZone(cv::Rect& rect, const cv::Mat& backProject)
  {
    if (rect.x < 0 || rect.y < 0)
      {
	rect.x = 0;
	rect.y = 0;
	rect.width = backProject.cols - 1;
	rect.height = backProject.rows - 1;
      }

    if (rect.x + rect.width > backProject.cols - 1)
      rect.width = backProject.cols - 1 - rect.x;
    if (rect.y + rect.height > backProject.rows - 1)
      rect.height = backProject.rows - 1 - rect.y;
  }
} // end of anonymous namespace.

boost::optional<cv::RotatedRect>
Object::track(const cv::Mat& image)
{

  if (algo == CAMSHIFT)
    return track_camshift(image);
  if (algo == NAIVE)
    return track_naive(image);
}

boost::optional<cv::RotatedRect>
Object::track_naive(const cv::Mat& image)
{
  cv::Mat imgHSV(image.rows, image.cols, CV_8UC3);
  cv::cvtColor(image, imgHSV, CV_BGR2HSV);
  cv::Mat  imgThreshed(image.rows, image.cols, CV_8UC1);
  cv::inRange(imgHSV, lower_hue,
              upper_hue, imgThreshed);
  std::vector< std::vector<cv::Point2i> > contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::dilate(imgThreshed, imgThreshed, cv::Mat() );
  cv::erode(imgThreshed, imgThreshed, cv::Mat() );
  cv::findContours(imgThreshed, contours,
                   CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE );
  typedef std::vector<cv::Point2i> Contour;
  Contour largest_contour;
  double max_area = 0;
  for (  std::vector<Contour>::iterator
           iter= contours.begin();
         iter != contours.end(); iter++ )
    {
      Contour contour = *iter;
      double area = cv::contourArea(cv::Mat(contour));
      if (area > max_area)
        {
          largest_contour = contour;
          max_area = area;
        }
    }
  cv::RotatedRect rrect;
  if (max_area > 0)
    {
      cv::Rect rect = cv::boundingRect(cv::Mat(largest_contour));
      rrect.center = cv::Point2f(rect.x + rect.width/2,
                                 rect.y + rect.height/2);
      rrect.size = cv::Size(rect.width, rect.height);
    }
  return rrect;
}

boost::optional<cv::RotatedRect>
Object::track_camshift(const cv::Mat& image)
{
  boost::optional<cv::RotatedRect> result;

  int nViews = modelHistogram.size();
  if (!nViews)
    return result;

  // Convert to HSV.
  cv::Mat hsv;
  cv::cvtColor(image, hsv, CV_BGR2HSV);

  // Compute back projection.
  //  only use channels 0 and 1 (hue and saturation).
  int channels[] = {0, 1};
  cv::Mat backProject;
  cv::calcBackProject(&hsv, 1, channels, modelHistogram[0], backProject,
		      ranges);

  for(int nmodel = 1; nmodel < nViews; ++nmodel)
    {
      cv::Mat backProjectTmp;
      cv::calcBackProject(&hsv, 1, channels, modelHistogram[nmodel],
			  backProjectTmp, ranges);

      // Merge back projections while taking care of overflows.
      for (int i = 0 ; i < backProject.rows; ++i)
	for (int j = 0 ; j < backProject.cols; ++j)
	  {
	    int v =
	      backProject.at<unsigned char>(i, j)
	      + backProjectTmp.at<unsigned char>(i, j);
	    if (v <= 0)
	      v = 0;
	    else if (v >= 255)
	      v = 255;
	    backProject.at<unsigned char>(i, j) = v;
	  }
    }

  cv::threshold(backProject, backProject, 32, 0, CV_THRESH_TOZERO);
  cv::medianBlur(backProject, backProject, 3);

  resetSearchZone(searchWindow_, backProject);

  cv::TermCriteria criteria =
    cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 50, 1);
  // std::cout << searchWindow_.width << " "
  //           << searchWindow_.height<< std::endl;
  result = cv::CamShift(backProject, searchWindow_, criteria);
  searchWindow_ = result->boundingRect();

  if (searchWindow_.height <= 0 || searchWindow_.width <= 0)
    searchWindow_.x = searchWindow_.y = -1;
  return result;
}
