#include "window_thread.h"
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread.hpp>

namespace hueblob {

void startWindowThread()
{
  static boost::once_flag cv_thread_flag = BOOST_ONCE_INIT;
  boost::call_once(cv_thread_flag, &cv::startWindowThread);
}

} // namespace hueblob
