#ifndef IMAGE_SOURCE_CAMERA_INFO_HPP
#define IMAGE_SOURCE_CAMERA_INFO_HPP

#include <string>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <image_transport/transport_hints.h>
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <boost/scoped_ptr.hpp>

namespace image_source {

class CameraInfo : public nodelet::Nodelet {
private:
  virtual void onInit() {
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());
    image_transport::ImageTransport it(nh);

    info_manager_.reset(new camera_info_manager::CameraInfoManager(
        nh, pnh.param< std::string >("camera_name", "camera"),
        pnh.param< std::string >("camera_info_url", "")));

    publisher_ = nh.advertise< sensor_msgs::CameraInfo >("camera_info", 1, true);

    const image_transport::TransportHints default_hints;
    subscriber_ = it.subscribe("image_in", 1, &CameraInfo::publish, this,
                               image_transport::TransportHints(default_hints.getTransport(),
                                                               default_hints.getRosHints(), pnh));
  }

  void publish(const sensor_msgs::ImageConstPtr &image) {
    const sensor_msgs::CameraInfoPtr info(
        new sensor_msgs::CameraInfo(info_manager_->getCameraInfo()));
    info->header = image->header;
    publisher_.publish(info);
  }

private:
  image_transport::Subscriber subscriber_;
  ros::Publisher publisher_;
  boost::scoped_ptr< camera_info_manager::CameraInfoManager > info_manager_;
};

} // namespace image_source

#endif