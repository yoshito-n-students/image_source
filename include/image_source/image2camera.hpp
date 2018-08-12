#ifndef IMAGE_SOURCE_IMAGE2CAMERA_HPP
#define IMAGE_SOURCE_IMAGE2CAMERA_HPP

#include <string>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <image_transport/transport_hints.h>
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <boost/make_shared.hpp>
#include <boost/scoped_ptr.hpp>

namespace image_source {

class Image2Camera : public nodelet::Nodelet {
private:
  virtual void onInit() {
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());
    image_transport::ImageTransport it(nh);

    info_manager_.reset(new camera_info_manager::CameraInfoManager(
        nh, pnh.param< std::string >("camera_name", "camera"),
        pnh.param< std::string >("camera_info_url", "")));

    // not "camera_out" because this is base name of the image topic.
    // name of camera info topic will be automatically generated in image_transport.
    publisher_ = it.advertiseCamera("image_out", 1, true);

    const image_transport::TransportHints default_hints;
    subscriber_ = it.subscribe("image_in", 1, &Image2Camera::publish, this,
                               image_transport::TransportHints(default_hints.getTransport(),
                                                               default_hints.getRosHints(), pnh));
  }

  void publish(const sensor_msgs::ImageConstPtr &image) {
    publisher_.publish(
        image, boost::make_shared< sensor_msgs::CameraInfo >(info_manager_->getCameraInfo()));
  }

private:
  image_transport::Subscriber subscriber_;
  image_transport::CameraPublisher publisher_;
  boost::scoped_ptr< camera_info_manager::CameraInfoManager > info_manager_;
};

} // namespace image_source

#endif