#ifndef IMAGE_SOURCE_VIDEO_FILE_HPP
#define IMAGE_SOURCE_VIDEO_FILE_HPP

#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/service_server.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <std_srvs/Empty.h>

#include <opencv2/videoio.hpp>

namespace image_source {

class VideoFile : public nodelet::Nodelet {
public:
  VideoFile() {}
  virtual ~VideoFile() {}

private:
  virtual void onInit() {
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());

    frame_id_ = pnh.param< std::string >("frame_id", "");
    encoding_ = pnh.param< std::string >("encoding", "bgr8");
    cv_type_ = cv_bridge::getCvType(encoding_);
    loop_ = pnh.param("loop", false);

    const std::string filename(pnh.param< std::string >("file", ""));
    if (filename.empty()) {
      NODELET_FATAL("No video filename");
      return;
    }
    if (!video_.open(filename)) {
      NODELET_FATAL_STREAM("Could not open " << filename);
      return;
    }

    frame_count_ = video_.get(cv::CAP_PROP_FRAME_COUNT);
    if (frame_count_ <= 0.) {
      NODELET_FATAL("Could not get number of video frames");
      return;
    }

    publisher_ = image_transport::ImageTransport(nh).advertise("image_out", 1, true);
    if (pnh.param("publish_by_call", false)) {
      server_ = nh.advertiseService("publish", &VideoFile::publishByCall, this);
    } else {
      const double original_fps(video_.get(cv::CAP_PROP_FPS));
      if (original_fps <= 0.) {
        NODELET_FATAL("Could not get fps from the video file");
        return;
      }
      const double playback_speed(pnh.param("playback_speed", 1.));
      if (playback_speed <= 0.) {
        NODELET_FATAL_STREAM("Invalid playback_speed: " << playback_speed);
        return;
      }
      timer_ = nh.createTimer(ros::Rate(original_fps * playback_speed), &VideoFile::publishByTimer,
                              this);
    }
  }

  bool publish() {
    cv_bridge::CvImage image;
    image.header.stamp = ros::Time::now();
    image.header.frame_id = frame_id_;
    image.encoding = encoding_;

    // playback from start if the video ends and loop option is enabled
    if (video_.get(cv::CAP_PROP_POS_FRAMES) == frame_count_) {
      if (loop_) {
        video_.set(cv::CAP_PROP_POS_FRAMES, 0.);
      } else {
        NODELET_INFO_ONCE("No more frames to be published");
        return false;
      }
    }

    // read the next frame
    video_.read(image.image);

    // validate the image
    if (image.image.empty()) {
      NODELET_ERROR("Could not read a frame");
      return false;
    }
    if (image.image.type() != cv_type_) {
      NODELET_ERROR("Unexpected image type");
      return false;
    }

    publisher_.publish(image.toImageMsg());

    return true;
  }

  void publishByTimer(const ros::TimerEvent &) { publish(); }

  bool publishByCall(std_srvs::Empty::Request &, std_srvs::Empty::Response &) { return publish(); }

private:
  std::string frame_id_, encoding_;
  int cv_type_;
  bool loop_;
  double frame_count_;

  ros::Timer timer_;
  ros::ServiceServer server_;
  image_transport::Publisher publisher_;

  cv::VideoCapture video_;
};

} // namespace image_source

#endif