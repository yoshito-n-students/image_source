# image_source
ROS nodelets to convert a local image file to a message

## Nodelet: ImageFiles
* publish image message from local image files

### Published topics
image_out (sensor_msgs/Image)

### Services
publish (std_srvs/Empty)
* publish the next image by a call
* never advertised unless ~publish_by_call is true

### Parameters
~loop (bool, default: false)
* loop publishment if true

~files (string[][2 or 3], default: \<empty array>)
* array of pairs of directory path and filename regular expression
* optionally image encoding to be published can be specified (default is "bgr8")

~recursive (bool, default: false)
* recursively search images if true

~frame_id (string, default: "")
* frame_id of published images

~publish_by_call (bool, default: false)
* publish by call or by timer

~rate (double, default: 1.0)
* publish rate of the timer

## Nodelet: VideoFile
* publish image messages from a local video file

### Published topics
image_out (sensor_msgs/Image)

### Services
publish (std_srvs/Empty)
* publish the next image by a call
* never advertised unless ~publish_by_call is true

### Parameters
~loop (bool, default: false)
* loop publishment if true

~file (string, default: "")
* path to video file

~frame_id (string, default: "")
* frame_id of image messages

~encoding (string, default: "bgr8")
* encoding of image messages

~publish_by_call (bool, default: false)
* publish by call or by timer

~playback_speed (double, default: 1.0)
* playback speed of video to set rate of timer

## Nodelet: CameraInfo
* publish camera calibration info synchronizing image messages
* requires full description of the published camera info

### Subscribed topics
image_in (sensor_msgs/Image)

### Published topics
camera_info (sensor_msgs/CameraInfo)
* the stamp of a message is copied from that of the subscribed image

### Services
set_camera_info (sensor_msgs/SetCameraInfo)

### Parameters
~camera_name (string, default: "camera")
* camera name used to load camera info

~camera_info_url (string, default: "")
* camera info on startup
* see docs of camera_info_manager for url syntax
* standard path will be searched if empty string is given

~frame_id (string, default: "")
* frame id of published camera info

## Nodelet: AutoCameraInfo
* publish camera calibration info synchronizing image messages
* requires only camera's field of view

### Subscribed topics
image_in (sensor_msgs/Image)

### Published topics
camera_info (sensor_msgs/CameraInfo)
* the stamp of a message is copied from that of the subscribed image

### Parameters
~frame_id (string, default: "")
* frame id of published camera info

~fov (double, required if ~fov_x or ~fov_y is not set)
* camera's diagonal field of view in radians

~fov_x, ~fov_y (double, optional)
* camera's horizontal or vertical field of view in radians

## Examples
see [launch/test_*.launch](launch)