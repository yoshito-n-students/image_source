#ifndef IMAGE_SOURCE_IMAGE_FILES_HPP
#define IMAGE_SOURCE_IMAGE_FILES_HPP

#include <queue>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <ros/timer.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <opencv2/highgui.hpp>

#include <boost/filesystem.hpp>
#include <boost/make_shared.hpp>
#include <boost/regex.hpp>

namespace image_source {

class ImageFiles : public nodelet::Nodelet {
public:
  ImageFiles() {}

  virtual ~ImageFiles() {}

private:
  virtual void onInit() {
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());

    loop_ = pnh.param("loop", true);

    loadDirectories(pnh.param("files", XmlRpc::XmlRpcValue()), pnh.param("recursive", false),
                    pnh.param< std::string >("frame_id", ""));
    if (queue_.empty()) {
      NODELET_FATAL("No image loaded");
      return;
    }

    publisher_ = image_transport::ImageTransport(nh).advertise("image_out", 1, true);
    timer_ = nh.createTimer(ros::Rate(pnh.param("rate", 1.)), &ImageFiles::publish, this);
  }

  void loadDirectories(const XmlRpc::XmlRpcValue &queries, const bool recursive,
                       const std::string &frame_id) {
    try {
      // iterate each directory query
      for (std::size_t i = 0; i < queries.size(); ++i) {
        // BOOST_FOREACH cannot be used because XmlRpcValue is not a regular sequence
        XmlRpc::XmlRpcValue query(queries[i]);
        const boost::filesystem::path directory(static_cast< std::string >(query[0]));
        const boost::regex filename_regex(static_cast< std::string >(query[1]));
        const std::string encoding(query.size() >= 3 ? static_cast< std::string >(query[2])
                                                     : std::string("bgr8"));
        loadDirectory(directory, recursive, filename_regex, frame_id, encoding);
      }
    } catch (const XmlRpc::XmlRpcException &error) {
      NODELET_ERROR_STREAM("Error in loading images: " << error.getMessage());
      return;
    } catch (const std::runtime_error &error) { // for boost.filesystem and regex
      NODELET_ERROR_STREAM("Error in loading images: " << error.what());
      return;
    }
  }

  void loadDirectory(const boost::filesystem::path &directory, const bool recursive,
                     const boost::regex &filename_regex, const std::string &frame_id,
                     const std::string &encoding) {
    namespace bf = boost::filesystem;
    namespace si = sensor_msgs::image_encodings;

    bf::directory_iterator entry(directory);
    bf::directory_iterator entry_end;
    const int cv_type(cv_bridge::getCvType(encoding));
    for (; entry != entry_end; ++entry) {
      const bf::path path(entry->path());

      // if directory
      if (bf::is_directory(path)) {
        if (recursive) {
          loadDirectory(path, recursive, filename_regex, frame_id, encoding);
        } else {
          NODELET_INFO_STREAM("Skip directory " << path);
        }
        continue;
      }

      // if regular file matching regex
      if (boost::regex_match(path.filename().string(), filename_regex)) {
        const cv::Mat image(cv::imread(path.string()));
        if (image.empty()) {
          NODELET_WARN_STREAM("Skip invalid image file: " << path);
          continue;
        }
        if (image.type() != cv_type) {
          NODELET_WARN_STREAM("Skip unexpected type image (depth or channels): " << path);
          continue;
        }
        std_msgs::Header header;
        header.frame_id = frame_id;
        queue_.push(boost::make_shared< cv_bridge::CvImage >(header, encoding, image));
        NODELET_INFO_STREAM("Loaded " << path);
        continue;
      }
    }
  }

  void publish(const ros::TimerEvent &) {
    if (queue_.empty()) {
      NODELET_INFO_ONCE("No more images to be published");
      return;
    }

    const cv_bridge::CvImagePtr image(queue_.front());
    queue_.pop();

    image->header.stamp = ros::Time::now();
    publisher_.publish(image->toImageMsg());

    if (loop_) {
      queue_.push(image);
    }
  }

private:
  bool loop_;

  std::queue< cv_bridge::CvImagePtr > queue_;

  ros::Timer timer_;
  image_transport::Publisher publisher_;
};

} // namespace image_source

#endif