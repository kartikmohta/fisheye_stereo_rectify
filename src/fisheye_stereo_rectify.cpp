#include "fisheye_stereo_rectify/fisheye_stereo_rectify.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>

using namespace fisheye_stereo_rectify;

namespace
{
template <typename T>
T get_param(const ros::NodeHandle &nh, const std::string &name)
{
  T val;
  if(!nh.getParam(name, val))
  {
    throw(std::runtime_error("cannot find parameter " + name));
  }
  return (val);
}
}

FisheyeStereoRectify::FisheyeStereoRectify(ros::NodeHandle &nh)
{
  image_transport::ImageTransport it(nh);
  for(int i = 0; i < 2; ++i)
  {
    const std::string cam_name = "cam" + std::to_string(i);

    pub_rect_[i] = it.advertise(cam_name + "/image_rect", 1);
    sub_image_[i] =
        it.subscribe(cam_name + "/image", 2,
                     boost::bind(&FisheyeStereoRectify::image_cb, this, _1, i));

    auto K_vec = get_param<std::vector<double>>(nh, cam_name + "/intrinsics");
    setK(K_vec, K_[i]);

    D_[i] = get_param<std::vector<double>>(nh, cam_name + "/distortion_coeffs");

    auto res_vec = get_param<std::vector<int>>(nh, cam_name + "/resolution");
    res_[i] = cv::Size(res_vec.at(0), res_vec.at(1));
  }

  auto T_vec = get_param<std::vector<double>>(nh, "cam1/T_cn_cnm1");
  setTransform(T_vec, R_, t_vec_);

  initializeRectificationMaps();
}

void FisheyeStereoRectify::setK(const std::vector<double> &K_vec,
                                cv::Matx33d &K)
{
  K = cv::Matx33d::eye();
  K(0, 0) = K_vec.at(0);
  K(1, 1) = K_vec.at(1);
  K(0, 2) = K_vec.at(2);
  K(1, 2) = K_vec.at(3);
}

void FisheyeStereoRectify::setTransform(const std::vector<double> &T_vec,
                                        cv::Matx33d &R, cv::Vec3d &t_vec)
{
  R(0, 0) = T_vec.at(0);
  R(0, 1) = T_vec.at(1);
  R(0, 2) = T_vec.at(2);
  R(1, 0) = T_vec.at(4);
  R(1, 1) = T_vec.at(5);
  R(1, 2) = T_vec.at(6);
  R(2, 0) = T_vec.at(8);
  R(2, 1) = T_vec.at(9);
  R(2, 2) = T_vec.at(10);

  t_vec(0) = T_vec.at(3);
  t_vec(1) = T_vec.at(7);
  t_vec(2) = T_vec.at(11);
}


void FisheyeStereoRectify::initializeRectificationMaps()
{
  cv::Mat R_rect[2];
  // std::cout << "K1:\n" << K_[0] << "\n";
  // std::cout << "K2:\n" << K_[1] << "\n";
  // std::cout << "D1:\n";
  // for(const auto &d : D_[0])
  //   std::cout << d << ", ";
  // std::cout << "\n";
  // std::cout << "D2:\n";
  // for(const auto &d : D_[1])
  //   std::cout << d << ", ";
  // std::cout << "\n";
  // std::cout << "res:\n" << res_[0] << "\n";
  // std::cout << "R_:\n" << R_ << "\n";
  // std::cout << "t:\n" << t_vec_ << "\n";
  cv::fisheye::stereoRectify(K_[0], D_[0], K_[1], D_[1], res_[0], R_, t_vec_,
                             R_rect[0], R_rect[1], P_[0], P_[1],
                             cv::noArray(), CV_CALIB_ZERO_DISPARITY);

  // std::cout << "P[0]:\n" << P_[0] << std::endl;
  // std::cout << "P[1]:\n" << P_[1] << std::endl;

  for(int i = 0; i < 2; ++i)
    cv::fisheye::initUndistortRectifyMap(K_[i], D_[i], R_rect[i], P_[i],
                                         res_[0], CV_16SC2, rectify_map_[i][0],
                                         rectify_map_[i][1]);

  rectify_maps_initialized_ = true;
}

void FisheyeStereoRectify::image_cb(const sensor_msgs::Image::ConstPtr &msg,
                                    int cam_id)
{
  if(!rectify_maps_initialized_)
    ROS_WARN("Rectification maps are not initialized!");

  auto cv_img_ptr = cv_bridge::toCvShare(msg);

  cv::Mat dst;
  cv::remap(cv_img_ptr->image, dst, rectify_map_[cam_id][0],
            rectify_map_[cam_id][1], cv::INTER_LINEAR);

  cv_bridge::CvImage out_cv_img(cv_img_ptr->header, cv_img_ptr->encoding, dst);

  pub_rect_[cam_id].publish(out_cv_img.toImageMsg());
}
