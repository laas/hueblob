#include <boost/format.hpp>
#include <boost/optional.hpp>
#include <gtest/gtest.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "libhueblob/object.hh"

void trackObject(const std::string& viewFilename,
		 const std::string& frameFilename)
{
  Object object;
  cv::Mat view = cv::imread(viewFilename);
  cv::Mat image = cv::imread(frameFilename);
  object.addView(view);

  EXPECT_EQ(object.modelHistogram.size(), 1u);

  boost::optional<cv::RotatedRect> rrect = object.track(image);

  // Check that tracking succeed.
  EXPECT_TRUE(rrect);

  cv::Rect rect = rrect->boundingRect();

  boost::format fmt("Blob location: %dx%d+%dx%d");
  fmt % rect.x % rect.y % rect.width % rect.height;

  std::cout << fmt << std::endl;

  cv::Point top_left(rect.x, rect.y);
  cv::Point bottom_right(rect.x + rect.width, rect.y + rect.height);
  cv::rectangle(image, top_left, bottom_right,
		cv::Scalar(0, 0, 255, 255), CV_FILLED);

  boost::format filename("tracking_result_%s.png");
  filename % frameFilename;

  if(!cv::imwrite(filename.str(), image))
    std::cerr << "Failed to save image.";
}

void viewHistogram(const cv::MatND& hist, const std::string& filename)
{
  int hbins = Object::h_bins, sbins = Object::s_bins;
  int scale = 10;

  double maxVal = 0.;
  minMaxLoc(hist, 0, &maxVal, 0, 0);

  cv::Mat histImg = cv::Mat::zeros(sbins * scale, hbins * scale, CV_8UC3);

  for(int h = 0; h < hbins; ++h)
    for(int s = 0; s < sbins; ++s)
      {
	float binVal = hist.at<float>(h, s);
	int intensity = cvRound(binVal*255/maxVal);
	cv::rectangle(histImg, cv::Point(h * scale, s * scale),
		      cv::Point((h + 1) * scale - 1, (s + 1) * scale - 1),
		      cv::Scalar::all(intensity),
		      CV_FILLED);
      }

  if(!cv::imwrite(filename, histImg))
    std::cerr << "Failed to save image.";
}

TEST(TestSuite, simple)
{
  Object object;

  EXPECT_EQ(object.anchor_x, 0.);
  EXPECT_EQ(object.anchor_y, 0.);
  EXPECT_EQ(object.anchor_z, 0.);

  EXPECT_TRUE(object.modelHistogram.empty());
  EXPECT_EQ(object.modelHistogram.size(), 0u);
}

TEST(TestSuite, compute_mask)
{
  Object object;
  cv::Mat img = cv::imread("./data/door.png");
  cv::Mat mask = object.computeMask(img);

  if(!cv::imwrite("mask.png", mask))
    std::cerr << "Failed to save image.";
}

TEST(TestSuite, add_view)
{
  Object object;
  cv::Mat img = cvLoadImage("./data/models/door.png");
  object.addView(img);

  EXPECT_EQ(object.modelHistogram.size(), 1u);

  viewHistogram(object.modelHistogram[0], "hist.png");
}

TEST(TestSuite, track)
{
  trackObject("./data/models/door.png", "./data/frames/door-frame.png");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
