#include <opencv2/core.hpp>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

namespace fisheye_stereo_rectify
{
class FisheyeStereoRectify
{
 public:
  FisheyeStereoRectify(ros::NodeHandle &nh);

 private:
  void setK(const std::vector<double> &K_vec, cv::Matx33d &K);
  void setTransform(const std::vector<double> &T_vec, cv::Matx33d &R,
                    cv::Vec3d &t_vec);
  void initializeRectificationMaps();
  void image_cb(const sensor_msgs::Image::ConstPtr &msg, int cam_id);

  image_transport::Subscriber sub_image_[2];
  image_transport::Publisher pub_rect_[2];

  cv::Matx33d K_[2];
  std::vector<double> D_[2];
  cv::Matx33d R_;
  cv::Vec3d t_vec_;
  cv::Matx34d P_[2];
  cv::Size res_[2];
  cv::Mat rectify_map_[2][2];
  bool rectify_maps_initialized_ = false;
};
}
