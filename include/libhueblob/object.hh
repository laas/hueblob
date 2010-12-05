#ifndef HUEBLOB_OBJECT_HH
# define HUEBLOB_OBJECT_HH
# include <vector>
# include <boost/optional.hpp>
# include <opencv2/core/core.hpp>

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
  void addView(const cv::Mat& view);

  /// \brief Track the object in the current image.
  ///
  /// \param image track in which the object will be tracked.
  /// \return a rotated rectangle if the object has been successfully tracked,
  ///         otherwise nothing.
  boost::optional<cv::RotatedRect> track(const cv::Mat& image);


  /// \brief Compute image mask used for histogram computation.
  ///
  /// Used internally by addView. The non zero values of this image
  /// indicates pixel to will be taken into account during the
  /// histogram computation step.
  ///
  /// \param view reference to the view
  cv::Mat computeMask(const cv::Mat& model);

  /// \name Anchor
  /// \{
  double anchor_x;
  double anchor_y;
  double anchor_z;
  /// \}

  /// \brief Contains all the histograms associated with this object.
  std::vector<cv::MatND> modelHistogram;

  /// \brief Object search window.
  ///
  /// Where the object has been seen the last time it has been
  /// successfully tracked.
  cv::Rect searchWindow_;
};

#endif //! HUEBLOB_OBJECT_HH
