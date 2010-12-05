#ifndef HUEBLOB_OBJECT_HH
# define HUEBLOB_OBJECT_HH
# include <vector>

# include <opencv/cv.h>

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
