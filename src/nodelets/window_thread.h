#ifndef HUEBLOB_WINDOW_THREAD_H
#define HUEBLOB_WINDOW_THREAD_H

namespace hueblob {

// Makes absolutely sure we only start the OpenCV window thread once
void startWindowThread();

} // namespace hueblob

#endif
