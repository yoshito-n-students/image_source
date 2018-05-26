#ifndef IMAGE_SOURCE_IMAGE_FILES_HPP
#define IMAGE_SOURCE_IMAGE_FILES_HPP

#include <queue>
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
    recursive_ = pnh.param("recursive", false);
    frame_id_ = pnh.param< std::string >("frame_id", "");

    loadDirectories();

    publisher_ = image_transport::ImageTransport(nh).advertise("image_out", 1, true);

    timer_ = nh.createTimer(ros::Rate(pnh.param("rate", 1.)), &ImageFiles::publish, this);
  }

  void loadDirectories() {
    ros::NodeHandle &pnh(getPrivateNodeHandle());

    // load a parameter tree
    XmlRpc::XmlRpcValue paths_tree;
    if (!pnh.getParam("images", paths_tree)) {
      NODELET_ERROR_STREAM("Error in parsing parameter" << pnh.resolveName("images")
                                                        << ": No parameter");
      return;
    }

    // convert the parameter tree to value
    try {
      for (std::size_t i = 0; i < paths_tree.size(); ++i) {
        XmlRpc::XmlRpcValue &path_tree(paths_tree[i]);
        const boost::filesystem::path directory(static_cast< std::string >(path_tree[0]));
        const boost::regex filename_regex(static_cast< std::string >(path_tree[1]));
        loadDirectory(directory, filename_regex);
      }
    } catch (const XmlRpc::XmlRpcException &error) {
      NODELET_ERROR_STREAM("Error in parsing parameter " << pnh.resolveName("images") << ": "
                                                         << error.getMessage()
                                                         << ". Will use the default value.");
      return;
    }
  }

  void loadDirectory(const boost::filesystem::path &directory, const boost::regex &filename_regex) {
    namespace bf = boost::filesystem;

    bf::directory_iterator entry(directory);
    bf::directory_iterator entry_end;
    for (; entry != entry_end; ++entry) {
      const bf::path path(entry->path());

      // if directory
      if (bf::is_directory(path)) {
        if (recursive_) {
          loadDirectory(path, filename_regex);
        } else {
          NODELET_INFO_STREAM("Skip directory " << path);
        }
        continue;
      }

      // if regular file matching regex
      if (boost::regex_match(path.filename().string(), filename_regex)) {
        const cv::Mat image(cv::imread(path.string()));
        if (image.empty()) {
          NODELET_ERROR_STREAM("Not a valid image file: " << path);
          continue;
        }
        std_msgs::Header header;
        header.frame_id = frame_id_;
        // TODO: encoding as parameter
        queue_.push(cv_bridge::CvImage(header, "bgr8", image).toImageMsg());
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

    const sensor_msgs::ImageConstPtr msg(queue_.front());
    queue_.pop();

    publisher_.publish(msg);

    if (loop_) {
      queue_.push(msg);
    }
  }

private:
  bool loop_, recursive_;
  std::string frame_id_;

  std::queue< sensor_msgs::ImageConstPtr > queue_;

  ros::Timer timer_;
  image_transport::Publisher publisher_;
};

} // namespace image_source

#endif