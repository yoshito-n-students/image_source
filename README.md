# image_source
ROS nodelets to convert a local image file to a message

## Nodelet: ImageFiles
* publish image message from local image files

### Published topics
image_out (sensor_msgs/Image)

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

~rate (double, default: 1.0)
* publish rate

## Nodelet: VideoFile
* publish image messages from a local video file

### Published topics
image_out (sensor_msgs/Image)

### Parameters
~loop (bool, default: false)
* loop publishment if true

~file (string, default: "")
* path to video file

~frame_id (string, default: "")
* frame_id of image messages

~encoding (string, default: "bgr8")
* encoding of image messages

~playback_speed (double, default: 1.0)
* playback speed of video


## Examples
see [launch/test_*.launch](launch)