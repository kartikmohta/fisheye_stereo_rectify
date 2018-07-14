#include <ros/ros.h>
#include <fisheye_stereo_rectify/fisheye_stereo_rectify.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fisheye_stereo_rectify_node");

  ros::NodeHandle nh("~");

  fisheye_stereo_rectify::FisheyeStereoRectify rectifier(nh);

  ros::spin();

  return 0;
}
