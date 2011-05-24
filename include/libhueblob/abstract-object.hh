#ifndef ABSTRACT_OBJECT_HH
# define ABSTRACT_OBJECT_HH
# include <vector>
# include <boost/optional.hpp>
# include <opencv2/core/core.hpp>

/// \brief Define an object of the object database.
class AbstractObject
{
public:
  explicit AbstractObject(){};
  virtual boost::optional<cv::RotatedRect> track(const cv::Mat& image) = 0;
};

#endif //! ABSTRACT_OBJECT_HH
