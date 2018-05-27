# image_source
ROS nodelets to convert a local image file to a message

## Nodelet: ImageFiles
* publish image message from local image files

### Published topics
image_out (sensor_msgs/Image)

### Parameters
~loop (bool, default: false)
* loop publishment if true

~images (string[][2], default: \<empty array>)
* array of pairs of directory path and filename regular expression

~recursive (bool, default: false)
* recursively search images if true

~frame_id (string, default: "")
* frame_id of published images

~rate (double, default: 1.0)
* publish rate