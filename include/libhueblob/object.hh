#ifndef HUEBLOB_OBJECT_HH
# define HUEBLOB_OBJECT_HH
# include <vector>

# include <opencv/cv.h>

/// \brief Define a hue/saturation image pair.
struct HueSaturation
{
  /// \brief Copy hue and saturation channels
  ///        from the source image into two
  ///         separate images.
  ///
  /// \param image source image
  HueSaturation(const IplImage& image);

  /// \brief Hue component.
  cv::Ptr<IplImage> h;
  /// \brief Saturation component.
  cv::Ptr<IplImage> s;

  /// \brief C array of OpenCV images pointing to hue and saturation
  /// images (in this order).
  ///
  /// This structure is required by OpenCV histogram related
  /// functions.
  IplImage* planes[2];
};

/// \brief Define an object of the object database.
///
/// An object is recognized by storing its histogram
/// in the modelHistogram attribute.
///
/// A 3d offset called anchor is also provided to tune
/// the position of the 3d point associated with an object.
struct Object {
  static const int h_bins = 25;
  static const int s_bins = 25;

  explicit Object();


  /// \brief Build the view histogram and append it to modelHistogram.
  ///
  /// The histogram is done on the hue and saturation components on
  /// the view.
  ///
  /// \param view reference to the view
  void addView(const IplImage& view);


  /// \brief Compute image mask.
  ///
  /// Used internally by addView.
  ///
  /// \param view reference to the view
  cv::Ptr<IplImage> computeMask(const IplImage& model);

  /// \name Anchor
  /// \{
  double anchor_x;
  double anchor_y;
  double anchor_z;
  /// \}

  /// \brief Contains all the histograms associated with this object.
  std::vector<cv::Ptr<CvHistogram> > modelHistogram;
};

#endif //! HUEBLOB_OBJECT_HH
