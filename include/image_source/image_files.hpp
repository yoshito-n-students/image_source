#ifndef IMAGE_SOURCE_IMAGE_FILES_HPP
#define IMAGE_SOURCE_IMAGE_FILES_HPP

#include <queue>
#include <regex>
#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/service_server.h>
#include <ros/time.h>
#include <ros/timer.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <std_srvs/Empty.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <opencv2/highgui.hpp>

#include <boost/filesystem.hpp>
#include <boost/range/algorithm/copy.hpp>

namespace image_source {

class ImageFiles : public nodelet::Nodelet {
public:
  ImageFiles() {}

  virtual ~ImageFiles() {}

protected:
  virtual void onInit() {
    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &pnh = getPrivateNodeHandle();

    loop_ = pnh.param("loop", true);
    frame_id_ = pnh.param<std::string>("frame_id", "");

    const bool recursive = pnh.param("recursive", false);
    for (const ImageQuery &query : imageQueriesParam(pnh, "files", {})) {
      for (const cv_bridge::CvImagePtr &image : loadImages(query, recursive)) {
        queue_.push(image);
      }
    }

    publisher_ = image_transport::ImageTransport(nh).advertise("image_out", 1, true);
    if (pnh.param("publish_by_call", false)) {
      server_ = nh.advertiseService("publish", &ImageFiles::publishByCall, this);
    } else {
      timer_ = nh.createTimer(ros::Rate(pnh.param("rate", 1.)), &ImageFiles::publishByTimer, this);
    }
  }

  struct ImageQuery {
    boost::filesystem::path directory;
    std::regex filename_regex;
    std::string encoding;
  };

  std::vector<ImageQuery> imageQueriesParam(ros::NodeHandle &nh, const std::string &key,
                                            const std::vector<ImageQuery> &default_val) {
    XmlRpc::XmlRpcValue param;
    if (!nh.getParam(key, param)) {
      return default_val;
    }

    try {
      std::vector<ImageQuery> val;
      for (std::size_t i = 0; i < param.size(); ++i) {
        XmlRpc::XmlRpcValue &pi = param[i];
        val.push_back({static_cast<std::string>(pi[0]), std::regex(static_cast<std::string>(pi[1])),
                       pi.size() >= 3 ? static_cast<std::string>(pi[2]) : "bgr8"});
      }
      return val;
    } catch (const XmlRpc::XmlRpcException &error) {
      NODELET_ERROR_STREAM("Error in parsing " << nh.resolveName(key) << ": "
                                               << error.getMessage());
      return default_val;
    } catch (const std::runtime_error &error) { // for boost.filesystem and regex
      NODELET_ERROR_STREAM("Error in parsing " << nh.resolveName(key) << ": " << error.what());
      return default_val;
    }
  }

  std::vector<cv_bridge::CvImagePtr> loadImages(const ImageQuery &query, const bool recursive) {
    namespace bf = boost::filesystem;
    namespace si = sensor_msgs::image_encodings;

    bf::directory_iterator entry(query.directory);
    bf::directory_iterator entry_end;
    const int cv_type = cv_bridge::getCvType(query.encoding);
    std::vector<cv_bridge::CvImagePtr> images;
    for (; entry != entry_end; ++entry) {
      const bf::path path = entry->path();

      // if directory
      if (bf::is_directory(path)) {
        if (recursive) {
          boost::copy(loadImages({path, query.filename_regex, query.encoding}, recursive),
                      std::back_inserter(images));
        } else {
          NODELET_INFO_STREAM("Skip directory " << path);
        }
        continue;
      }

      // if regular file matching regex
      if (std::regex_match(path.filename().string(), query.filename_regex)) {
        const cv::Mat image = cv::imread(path.string());
        if (image.empty()) {
          NODELET_WARN_STREAM("Skip invalid image file: " << path);
          continue;
        }
        if (image.type() != cv_type) {
          NODELET_WARN_STREAM("Skip unexpected type image (depth or channels): " << path);
          continue;
        }
        images.push_back(cv_bridge::CvImagePtr(
            new cv_bridge::CvImage(std_msgs::Header(), query.encoding, image)));
        NODELET_INFO_STREAM("Loaded " << path);
        continue;
      }
    }
    return images;
  }

  bool publish() {
    if (queue_.empty()) {
      NODELET_INFO_ONCE("No more images to be published");
      return false;
    }

    const cv_bridge::CvImagePtr image(queue_.front());
    queue_.pop();

    image->header.stamp = ros::Time::now();
    image->header.frame_id = frame_id_;

    publisher_.publish(image->toImageMsg());

    if (loop_) {
      queue_.push(image);
    }

    return true;
  }

  void publishByTimer(const ros::TimerEvent &) { publish(); }

  bool publishByCall(std_srvs::Empty::Request &, std_srvs::Empty::Response &) { return publish(); }

protected:
  bool loop_;
  std::string frame_id_;

  std::queue<cv_bridge::CvImagePtr> queue_;

  ros::Timer timer_;
  ros::ServiceServer server_;
  image_transport::Publisher publisher_;
};

} // namespace image_source

#endif