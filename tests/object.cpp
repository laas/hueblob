#include <gtest/gtest.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "libhueblob/object.hh"

void viewHistogram(const CvHistogram& hist, const std::string& filename)
{
  int h_bins = Object::h_bins, s_bins = Object::s_bins;

  int scale = 10;
  cv::Ptr<IplImage> hist_img =
    cvCreateImage(cvSize(h_bins * scale, s_bins * scale), 8, 3);
  cvZero(hist_img);

  // populate the visualization
  float max_value = 0.;
  cvGetMinMaxHistValue(&hist, 0, &max_value, 0, 0);

  for(int h = 0; h < h_bins; ++h)
    {
      for(int s = 0; s < s_bins; ++s)
	{
	  float bin_val = cvQueryHistValue_2D(&hist, h, s);
	  int intensity = cvRound(bin_val * 255 / max_value);
	  cvRectangle(hist_img, cvPoint( h * scale, s * scale),
		      cvPoint((h + 1) * scale - 1, (s + 1) * scale - 1),
		      CV_RGB(intensity, intensity, intensity),
		      CV_FILLED);
	}
    }

  if(!cvSaveImage(filename.c_str(), hist_img))
    std::cerr << "Failed to save image.";
}

TEST(TestSuite, simple)
{
  Object object;

  EXPECT_EQ(object.anchor_x, 0.);
  EXPECT_EQ(object.anchor_y, 0.);
  EXPECT_EQ(object.anchor_z, 0.);

  EXPECT_TRUE(object.modelHistogram.empty());
  EXPECT_EQ(object.modelHistogram.size(), 0);
}

TEST(TestSuite, compute_mask)
{
  Object object;
  cv::Ptr<IplImage> img = cvLoadImage("./data/door.png");
  cv::Ptr<IplImage> mask = object.computeMask(*img);

  if(!cvSaveImage("mask.png", mask))
    std::cerr << "Failed to save image.";
}

TEST(TestSuite, add_view)
{
  Object object;
  cv::Ptr<IplImage> img = cvLoadImage("./data/door.png");
  object.addView(*img);

  EXPECT_EQ(object.modelHistogram.size(), 1);

  viewHistogram(*object.modelHistogram[0], "hist.png");
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
