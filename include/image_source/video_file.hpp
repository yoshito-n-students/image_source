#ifndef IMAGE_SOURCE_VIDEO_FILE_HPP
#define IMAGE_SOURCE_VIDEO_FILE_HPP

#include <nodelet/nodelet.h>

namespace image_source {

class VideoFile : public nodelet::Nodelet {
public:
  VideoFile() {}
  virtual ~VideoFile() {}

private:
  virtual void onInit() {}
};

} // namespace image_source

#endif