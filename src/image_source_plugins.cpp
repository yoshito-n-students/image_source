#include <image_source/camera_info.hpp>
#include <image_source/image_files.hpp>
#include <image_source/video_file.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(image_source::CameraInfo, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(image_source::ImageFiles, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(image_source::VideoFile, nodelet::Nodelet);