#ifndef IMAGE_SOURCE_AUTO_CAMERA_INFO_HPP
#define IMAGE_SOURCE_AUTO_CAMERA_INFO_HPP

#include <cmath>
#include <string>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber.h>
#include <image_transport/transport_hints.h>
#include <nodelet/nodelet.h>
#include <ros/exception.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/distortion_models.h>

#include <boost/optional.hpp>

namespace image_source {

class AutoCameraInfo : public nodelet::Nodelet {
public:
  AutoCameraInfo() {}

  virtual ~AutoCameraInfo() {}

protected:
  virtual void onInit() {
    ros::NodeHandle &nh(getNodeHandle());
    ros::NodeHandle &pnh(getPrivateNodeHandle());
    image_transport::ImageTransport it(nh);

    frame_id_ = pnh.param< std::string >("frame_id", "");
    fov_ = optionalParam< double >(pnh, "fov");
    fov_x_ = optionalParam< double >(pnh, "fov_x");
    fov_y_ = optionalParam< double >(pnh, "fov_y");
    if (!fov_ && !fov_x_) {
      throw ros::Exception("AutoCameraInfo::onInit(): '" + pnh.resolveName("fov") + "' or '" +
                           pnh.resolveName("fov_x") + "' must be given");
    }
    if (!fov_ && !fov_y_) {
      throw ros::Exception("AutoCameraInfo::onInit(): '" + pnh.resolveName("fov") + "' or '" +
                           pnh.resolveName("fov_y") + "' must be given");
    }

    publisher_ = nh.advertise< sensor_msgs::CameraInfo >("camera_info", 1, true);

    const image_transport::TransportHints default_hints;
    subscriber_ = it.subscribe("image_in", 1, &AutoCameraInfo::publish, this,
                               image_transport::TransportHints(default_hints.getTransport(),
                                                               default_hints.getRosHints(), pnh));
  }

  void publish(const sensor_msgs::ImageConstPtr &image) {
    const sensor_msgs::CameraInfoPtr info(new sensor_msgs::CameraInfo());
    // header
    info->header.stamp = image->header.stamp;
    info->header.frame_id = frame_id_;
    // image size
    info->height = image->height;
    info->width = image->width;
    // distortion params (no distortion)
    info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    info->D.resize(5, 0.);
    // intrinsic camera matrix (assuming the principal point is the image center)
    const double cx(info->width / 2.), cy(info->height / 2.);
    const double fx(fov_x_ ? cx / std::tan(*fov_x_ / 2.)
                           : std::sqrt(cx * cx + cy * cy) / std::tan(*fov_ / 2.));
    const double fy(fov_y_ ? cy / std::tan(*fov_y_ / 2.)
                           : std::sqrt(cx * cx + cy * cy) / std::tan(*fov_ / 2.));
    info->K[0] = fx;
    info->K[1] = 0.;
    info->K[2] = cx;
    info->K[3] = 0.;
    info->K[4] = fy;
    info->K[5] = cy;
    info->K[6] = 0.;
    info->K[7] = 0.;
    info->K[8] = 1.;
    // rectification matrix
    // (not required for a monocular camera but set identity matrix just in case)
    info->R[0] = 1.;
    info->R[1] = 0.;
    info->R[2] = 0.;
    info->R[3] = 0.;
    info->R[4] = 1.;
    info->R[5] = 0.;
    info->R[6] = 0.;
    info->R[7] = 0.;
    info->R[8] = 1.;
    // projection matrix
    // (almost same as the intrinsic camera matrix in case of a monocular camera)
    info->P[0] = fx;
    info->P[1] = 0.;
    info->P[2] = cx;
    info->P[3] = 0.;
    info->P[4] = 0.;
    info->P[5] = fy;
    info->P[6] = cy;
    info->P[7] = 0.;
    info->P[8] = 0.;
    info->P[9] = 0.;
    info->P[10] = 1.;
    info->P[11] = 0.;
    // leave remaining params (binning and region of interest) as their default

    publisher_.publish(info);
  }

  template < typename T >
  static boost::optional< T > optionalParam(ros::NodeHandle &nh, const std::string &key) {
    T val;
    return boost::make_optional(nh.getParam(key, val), val);
  }

protected:
  std::string frame_id_;
  boost::optional< double > fov_, fov_x_, fov_y_;

  ros::Publisher publisher_;
  image_transport::Subscriber subscriber_;
};

} // namespace image_source

#endif