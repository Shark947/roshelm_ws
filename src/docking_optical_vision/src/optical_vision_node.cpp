#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>

#include <docking_optical_msgs/OpticalMeasurement.h>
#include <common_msgs/Float64Stamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <cmath>
#include <limits>
#include <string>

class OpticalVisionNode
{
public:
  OpticalVisionNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : it_(nh)
  {
    // ====== 1) ROS 参数读取 ======
    pnh.param<std::string>("camera_topic", camera_topic_, std::string("/camera/image"));
    pnh.param<std::string>("camera_info_topic", camera_info_topic_, std::string("/camera/camera_info"));
    pnh.param<std::string>("optical_measurement_topic", meas_topic_, std::string("/docking/optical_measurement"));
    pnh.param<std::string>("optical_feedback_topic", feedback_topic_, std::string("/docking/optical_feedback"));
    pnh.param<std::string>("nav_depth_topic", nav_depth_topic_, std::string("/auh/NAV_DEPTH"));
    pnh.param<std::string>("nav_heading_topic", nav_heading_topic_, std::string("/auh/NAV_HEADING"));
    pnh.param<std::string>("dock_depth_topic", dock_depth_topic_, std::string("/docking/dock_depth"));

    pnh.param<int>("queue_size", queue_size_, 1);

    pnh.param<int>("print_every_n", print_every_n_, 30);
    pnh.param<int>("print_camera_info_every_n", print_caminfo_every_n_, 120);
    pnh.param<int>("print_detection_every_n", print_detection_every_n_, 10);
    pnh.param<int>("min_consecutive_detections", min_consecutive_detections_, 2);
    pnh.param<int>("max_consecutive_misses", max_consecutive_misses_, 2);
    pnh.param<double>("optical_timeout_sec", optical_timeout_sec_, 0.5);

    // ROI
    pnh.param<bool>("use_roi", use_roi_, false);
    pnh.param<int>("roi_x", roi_x_, 0);
    pnh.param<int>("roi_y", roi_y_, 0);
    pnh.param<int>("roi_w", roi_w_, 0);
    pnh.param<int>("roi_h", roi_h_, 0);

    // preprocess
    pnh.param<int>("blur_kernel", blur_kernel_, 3);

    // threshold
    pnh.param<int>("threshold_value", threshold_value_, 240);
    pnh.param<bool>("use_adaptive_threshold", use_adaptive_threshold_, false);
    pnh.param<int>("adaptive_block_size", adaptive_block_size_, 31);
    pnh.param<int>("adaptive_c", adaptive_c_, -5);

    // morphology
    pnh.param<int>("morph_open_kernel", morph_open_kernel_, 3);
    pnh.param<int>("morph_close_kernel", morph_close_kernel_, 5);

    // blob filter
    pnh.param<int>("min_blob_area", min_blob_area_, 5);
    pnh.param<int>("max_blob_area", max_blob_area_, 5000);
    pnh.param<double>("min_brightness", min_brightness_, 200.0);
    pnh.param<double>("max_pixel_jump", max_pixel_jump_, 80.0);
    pnh.param<double>("ema_alpha", ema_alpha_, 0.4);
    pnh.param<std::string>("choose_mode", choose_mode_, std::string("brightest"));
    pnh.param<double>("optical_max_theta_deg", optical_max_theta_deg_, 30.0);
    pnh.param<double>("optical_depth_min_m", optical_depth_min_m_, 0.3);
    pnh.param<double>("optical_depth_max_m", optical_depth_max_m_, 12.0);
    pnh.param<double>("optical_max_next_xy_jump_m", optical_max_next_xy_jump_m_, 3.0);

    pnh.param<double>("optical_camera_delta_l", optical_camera_delta_l_, 0.0);
    pnh.param<double>("depth_bias", depth_bias_, 0.0);
    pnh.param<double>("depth_camera_bias", depth_camera_bias_, 0.0);
    pnh.param<double>("dock_panel", dock_panel_, 0.0);

    // sign
    pnh.param<bool>("invert_theta_x", invert_theta_x_, false);
    pnh.param<bool>("invert_theta_y", invert_theta_y_, false);

    // 参数安全修正：自适应阈值 block_size 必须奇数且 >=3
    if (adaptive_block_size_ < 3) adaptive_block_size_ = 3;
    if (adaptive_block_size_ % 2 == 0) adaptive_block_size_ += 1;
    if (blur_kernel_ < 0) blur_kernel_ = 0;
    if (blur_kernel_ % 2 == 0 && blur_kernel_ != 0) blur_kernel_ += 1;
    if (ema_alpha_ < 0.0) ema_alpha_ = 0.0;
    if (ema_alpha_ > 1.0) ema_alpha_ = 1.0;

    // ====== 2) 订阅 ======
    img_sub_ = it_.subscribe(camera_topic_, queue_size_, &OpticalVisionNode::imageCb, this);
    caminfo_sub_ = nh.subscribe(camera_info_topic_, 1, &OpticalVisionNode::cameraInfoCb, this);
    nav_depth_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
        nav_depth_topic_, 10, &OpticalVisionNode::navDepthCb, this);
    nav_heading_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
        nav_heading_topic_, 10, &OpticalVisionNode::navHeadingCb, this);
    dock_depth_sub_ = nh.subscribe<common_msgs::Float64Stamped>(
        dock_depth_topic_, 10, &OpticalVisionNode::dockDepthCb, this);

    // ====== 3) 发布 ======
    meas_pub_ = nh.advertise<docking_optical_msgs::OpticalMeasurement>(meas_topic_, 10);
    if (optical_timeout_sec_ > 0.0)
    {
      timeout_timer_ = nh.createTimer(ros::Duration(0.1), &OpticalVisionNode::timeoutCb, this);
    }

    ROS_INFO_STREAM("[docking_optical_vision] camera_topic=" << camera_topic_);
    ROS_INFO_STREAM("[docking_optical_vision] camera_info_topic=" << camera_info_topic_);
    ROS_INFO_STREAM("[docking_optical_vision] publish measurement=" << meas_topic_);
    ROS_INFO_STREAM("[docking_optical_vision] choose_mode=" << choose_mode_
                    << " thr=" << threshold_value_
                    << " min_brightness=" << min_brightness_
                    << " max_pixel_jump=" << max_pixel_jump_
                    << " ema_alpha=" << ema_alpha_
                    << " invert_x=" << invert_theta_x_ << " invert_y=" << invert_theta_y_);
    ROS_INFO_STREAM("[docking_optical_vision] nav_depth_topic=" << nav_depth_topic_
                    << " nav_heading_topic=" << nav_heading_topic_
                    << " dock_depth_topic=" << dock_depth_topic_);
    ROS_INFO_STREAM("[docking_optical_vision] optical_limits: theta_deg=" << optical_max_theta_deg_
                    << " depth=[" << optical_depth_min_m_ << ","
                    << optical_depth_max_m_ << "] max_next_xy_jump="
                    << optical_max_next_xy_jump_m_ << " timeout_sec="
                    << optical_timeout_sec_);
  }

private:
  void navDepthCb(const common_msgs::Float64Stamped::ConstPtr& msg)
  {
    nav_depth_ = msg->data;
    has_nav_depth_ = true;
  }

  void navHeadingCb(const common_msgs::Float64Stamped::ConstPtr& msg)
  {
    nav_heading_deg_ = msg->data;
    has_nav_heading_ = true;
  }

  void dockDepthCb(const common_msgs::Float64Stamped::ConstPtr& msg)
  {
    dock_depth_ = msg->data;
    has_dock_depth_ = true;
  }

  void timeoutCb(const ros::TimerEvent&)
  {
    if (last_frame_time_.isZero())
    {
      return;
    }
    const ros::Time now = ros::Time::now();
    if ((now - last_frame_time_).toSec() <= optical_timeout_sec_)
    {
      return;
    }
    if ((now - last_timeout_publish_).toSec() < optical_timeout_sec_)
    {
      return;
    }
    last_timeout_publish_ = now;
    filtered_valid_ = false;
    consecutive_detects_ = 0;
    consecutive_misses_ = max_consecutive_misses_;
    std_msgs::Header header;
    header.stamp = now;
    header.frame_id = last_frame_id_;
    publishMeasurement(header, 0.0, 0.0);
  }

  // ===== 相机内参 =====
  void cameraInfoCb(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    caminfo_count_++;

    fx_ = msg->K[0];
    fy_ = msg->K[4];
    cx_ = msg->K[2];
    cy_ = msg->K[5];
    has_caminfo_ = true;

    if (print_caminfo_every_n_ > 0 && (caminfo_count_ % print_caminfo_every_n_ == 0))
    {
      ROS_INFO_STREAM("[vision] CameraInfo frame_id=" << msg->header.frame_id
                      << " fx=" << fx_ << " fy=" << fy_
                      << " cx=" << cx_ << " cy=" << cy_);
    }
  }

  // ===== 核心：图像回调 -> 白点检测 -> theta -> 发布测量 =====
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    frame_count_++;
    last_frame_time_ = msg->header.stamp;
    last_frame_id_ = msg->header.frame_id;

    if (print_every_n_ > 0 && (frame_count_ % print_every_n_ == 0))
    {
      ROS_INFO_STREAM("[vision] frame=" << frame_count_
                      << " stamp=" << msg->header.stamp
                      << " frame_id=" << msg->header.frame_id
                      << " " << msg->width << "x" << msg->height
                      << " encoding=" << msg->encoding);
    }

    // 没有 CameraInfo 时仍然可以做“检测”，但无法算 theta（这里直接提示并发布 valid=false）
    if (!has_caminfo_)
    {
      ROS_WARN_THROTTLE(2.0, "[vision] No CameraInfo yet. Publish valid=false.");
      markDetectionResult(false);
      publishMeasurement(msg->header, 0.0, 0.0);
      return;
    }

    // ---- 1) ROS Image -> cv::Mat ----
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      // 你的图像是 rgb8（日志里看到了），直接按 rgb8 转
      cv_ptr = cv_bridge::toCvShare(msg, "rgb8");
    }
    catch (const cv_bridge::Exception& e)
    {
      ROS_ERROR_STREAM("[vision] cv_bridge exception: " << e.what());
      markDetectionResult(false);
      publishMeasurement(msg->header, 0.0, 0.0);
      return;
    }

    cv::Mat rgb = cv_ptr->image;
    if (rgb.empty())
    {
      markDetectionResult(false);
      publishMeasurement(msg->header, 0.0, 0.0);
      return;
    }

    // ---- 2) ROI（可选）----
    cv::Rect roi(0, 0, rgb.cols, rgb.rows);
    if (use_roi_)
    {
      // 防越界裁剪
      int x = std::max(0, roi_x_);
      int y = std::max(0, roi_y_);
      int w = (roi_w_ > 0) ? roi_w_ : rgb.cols;
      int h = (roi_h_ > 0) ? roi_h_ : rgb.rows;
      w = std::min(w, rgb.cols - x);
      h = std::min(h, rgb.rows - y);
      roi = cv::Rect(x, y, w, h);
    }
    cv::Mat rgb_roi = rgb(roi);

    // ---- 3) 转灰度 ----
    cv::Mat gray;
    cv::cvtColor(rgb_roi, gray, cv::COLOR_RGB2GRAY);

    // ---- 4) 去噪（可选）----
    if (blur_kernel_ >= 3)
      cv::GaussianBlur(gray, gray, cv::Size(blur_kernel_, blur_kernel_), 0);

    // ---- 5) 阈值分割：得到二值白点 ----
    cv::Mat bin;
    if (use_adaptive_threshold_)
    {
      cv::adaptiveThreshold(
        gray, bin, 255,
        cv::ADAPTIVE_THRESH_MEAN_C,
        cv::THRESH_BINARY,
        adaptive_block_size_,
        adaptive_c_);
    }
    else
    {
      cv::threshold(gray, bin, threshold_value_, 255, cv::THRESH_BINARY);
    }

    // ---- 6) 形态学（可选）----
    if (morph_open_kernel_ >= 3)
    {
      cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                            cv::Size(morph_open_kernel_, morph_open_kernel_));
      cv::morphologyEx(bin, bin, cv::MORPH_OPEN, k);
    }
    if (morph_close_kernel_ >= 3)
    {
      cv::Mat k = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                            cv::Size(morph_close_kernel_, morph_close_kernel_));
      cv::morphologyEx(bin, bin, cv::MORPH_CLOSE, k);
    }

    // ---- 7) 连通域/轮廓找白点 ----
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(bin, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty())
    {
      // 没检测到白点
      markDetectionResult(false);
      publishMeasurement(msg->header, 0.0, 0.0);
      return;
    }

    // ---- 8) 选择目标：largest 或 brightest ----
    int best_idx = -1;
    double best_score = -1.0;

    for (int i = 0; i < (int)contours.size(); ++i)
    {
      double area = cv::contourArea(contours[i]);
      if (area < min_blob_area_ || area > max_blob_area_)
        continue;

      // 质心
      cv::Moments mu = cv::moments(contours[i]);
      if (mu.m00 <= 1e-6) continue;
      double u = mu.m10 / mu.m00;
      double v = mu.m01 / mu.m00;

      cv::Rect bb = cv::boundingRect(contours[i]);
      bb &= cv::Rect(0, 0, gray.cols, gray.rows);
      cv::Scalar mean_val = cv::mean(gray(bb), bin(bb));
      double brightness = mean_val[0];
      if (brightness < min_brightness_)
        continue;

      double score = 0.0;
      if (choose_mode_ == "largest")
      {
        score = area;  // 面积越大越优
      }
      else
      {
        // brightest：以灰度均值作为亮度评分（更稳，避免“噪点大但不亮”）
        score = brightness;
      }

      if (score > best_score)
      {
        best_score = score;
        best_idx = i;
      }
    }

    if (best_idx < 0)
    {
      markDetectionResult(false);
      publishMeasurement(msg->header, 0.0, 0.0);
      return;
    }

    // ---- 9) 计算最终像素中心(u,v)，注意加回 ROI 偏移 ----
    cv::Moments mu = cv::moments(contours[best_idx]);
    double u_roi = mu.m10 / mu.m00;
    double v_roi = mu.m01 / mu.m00;

    // 图像坐标系下的像素中心（相对于整幅图）
    double u = u_roi + roi.x;
    double v = v_roi + roi.y;

    if (has_last_detection_ && max_pixel_jump_ > 0.0)
    {
      double dist = std::hypot(u - last_u_, v - last_v_);
      if (dist > max_pixel_jump_)
      {
        markDetectionResult(false);
        publishMeasurement(msg->header, 0.0, 0.0);
        return;
      }
    }

    // ---- 10) 像素 -> 角度（deg）----
    // optical image 坐标默认：
    //   u 向右为正，v 向下为正
    // theta_x = atan((u-cx)/fx), theta_y = atan((v-cy)/fy)
    double theta_x = std::atan((u - cx_) / fx_) * 180.0 / M_PI;
    double theta_y = std::atan((v - cy_) / fy_) * 180.0 / M_PI;

    if (invert_theta_x_) theta_x = -theta_x;
    if (invert_theta_y_) theta_y = -theta_y;

    if (optical_max_theta_deg_ > 0.0 &&
        (std::abs(theta_x) > optical_max_theta_deg_ ||
         std::abs(theta_y) > optical_max_theta_deg_))
    {
      markDetectionResult(false);
      publishMeasurement(msg->header, 0.0, 0.0);
      return;
    }

    if (has_last_theta_)
    {
      theta_x = ema_alpha_ * theta_x + (1.0 - ema_alpha_) * last_theta_x_;
      theta_y = ema_alpha_ * theta_y + (1.0 - ema_alpha_) * last_theta_y_;
    }
    last_theta_x_ = theta_x;
    last_theta_y_ = theta_y;
    has_last_theta_ = true;
    last_u_ = u;
    last_v_ = v;
    has_last_detection_ = true;
    if (!opticalGeometryValid(theta_x, theta_y))
    {
      markDetectionResult(false);
      publishMeasurement(msg->header, 0.0, 0.0);
      return;
    }

    // ---- 11) 发布 OpticalMeasurement ----
    markDetectionResult(true);
    publishMeasurement(msg->header, theta_x, theta_y);

    // ---- 12) 控制台输出（节流）----
    detect_count_++;
    if (print_detection_every_n_ > 0 && (detect_count_ % print_detection_every_n_ == 0))
    {
      double area = cv::contourArea(contours[best_idx]);
      ROS_INFO_STREAM("[vision] DETECT u=" << u << " v=" << v
                      << " area=" << area
                      << " theta_x_deg=" << theta_x
                      << " theta_y_deg=" << theta_y);
    }
  }

  void markDetectionResult(bool detected)
  {
    if (detected)
    {
      ++consecutive_detects_;
      consecutive_misses_ = 0;
      if (consecutive_detects_ >= min_consecutive_detections_)
      {
        filtered_valid_ = true;
      }
    }
    else
    {
      ++consecutive_misses_;
      consecutive_detects_ = 0;
      if (consecutive_misses_ >= max_consecutive_misses_)
      {
        filtered_valid_ = false;
      }
    }
  }

  void publishMeasurement(const std_msgs::Header& header, double theta_x,
                          double theta_y)
  {
    docking_optical_msgs::OpticalMeasurement meas;
    meas.header = header;
    meas.valid = filtered_valid_;
    meas.d_heading_deg = 0.0;
    meas.theta_x_deg = filtered_valid_ ? theta_x : 0.0;
    meas.theta_y_deg = filtered_valid_ ? theta_y : 0.0;
    meas.fallback_x = 0.0;
    meas.fallback_y = 0.0;
    meas_pub_.publish(meas);
  }

  bool opticalGeometryValid(double theta_x, double theta_y)
  {
    if (optical_depth_min_m_ <= 0.0 && optical_depth_max_m_ <= 0.0 &&
        optical_max_next_xy_jump_m_ <= 0.0)
    {
      return true;
    }

    if (has_nav_depth_ && has_dock_depth_)
    {
      const double light_depth = dock_depth_ - dock_panel_;
      const double camera_depth = nav_depth_ + depth_bias_ + depth_camera_bias_;
      const double depth_delta = light_depth - camera_depth;
      if (!std::isfinite(depth_delta))
      {
        return false;
      }
      if (optical_depth_min_m_ > 0.0 && depth_delta < optical_depth_min_m_)
      {
        return false;
      }
      if (optical_depth_max_m_ > 0.0 && depth_delta > optical_depth_max_m_)
      {
        return false;
      }

      if (optical_max_next_xy_jump_m_ > 0.0 && has_nav_heading_)
      {
        const double heading_rad = nav_heading_deg_ * M_PI / 180.0;
        const double dx = depth_delta * std::tan(theta_x * M_PI / 180.0) *
                              std::sin(heading_rad) -
                          depth_delta * std::tan(theta_y * M_PI / 180.0) *
                              std::cos(heading_rad) -
                          optical_camera_delta_l_ * std::sin(heading_rad);
        const double dy = -depth_delta * std::tan(theta_y * M_PI / 180.0) *
                              std::sin(heading_rad) -
                          depth_delta * std::tan(theta_x * M_PI / 180.0) *
                              std::cos(heading_rad) +
                          optical_camera_delta_l_ * std::cos(heading_rad);
        const double next_x = dy;
        const double next_y = dx;
        if (has_last_next_xy_)
        {
          const double jump = std::hypot(next_x - last_next_x_,
                                         next_y - last_next_y_);
          if (jump > optical_max_next_xy_jump_m_)
          {
            return false;
          }
        }
        has_last_next_xy_ = true;
        last_next_x_ = next_x;
        last_next_y_ = next_y;
      }
    }
    return true;
  }

private:
  image_transport::ImageTransport it_;
  image_transport::Subscriber img_sub_;
  ros::Subscriber caminfo_sub_;
  ros::Subscriber nav_depth_sub_;
  ros::Subscriber nav_heading_sub_;
  ros::Subscriber dock_depth_sub_;
  ros::Publisher meas_pub_;
  ros::Timer timeout_timer_;

  std::string camera_topic_;
  std::string camera_info_topic_;
  std::string meas_topic_;
  std::string feedback_topic_;
  std::string nav_depth_topic_;
  std::string nav_heading_topic_;
  std::string dock_depth_topic_;

  int queue_size_{1};

  int print_every_n_{30};
  int print_caminfo_every_n_{120};
  int print_detection_every_n_{10};
  int min_consecutive_detections_{2};
  int max_consecutive_misses_{2};
  double optical_timeout_sec_{0.5};

  // ROI params
  bool use_roi_{false};
  int roi_x_{0}, roi_y_{0}, roi_w_{0}, roi_h_{0};

  // preprocess params
  int blur_kernel_{3};

  // threshold params
  int threshold_value_{240};
  bool use_adaptive_threshold_{false};
  int adaptive_block_size_{31};
  int adaptive_c_{-5};

  // morphology params
  int morph_open_kernel_{3};
  int morph_close_kernel_{5};

  // blob filter
  int min_blob_area_{5};
  int max_blob_area_{5000};
  double min_brightness_{200.0};
  double max_pixel_jump_{80.0};
  double ema_alpha_{0.4};
  std::string choose_mode_{"brightest"};
  double optical_max_theta_deg_{30.0};
  double optical_depth_min_m_{0.3};
  double optical_depth_max_m_{12.0};
  double optical_max_next_xy_jump_m_{3.0};

  double optical_camera_delta_l_{0.0};
  double depth_bias_{0.0};
  double depth_camera_bias_{0.0};
  double dock_panel_{0.0};

  // sign convention
  bool invert_theta_x_{false};
  bool invert_theta_y_{false};

  uint64_t frame_count_{0};
  uint64_t caminfo_count_{0};
  uint64_t detect_count_{0};

  int consecutive_detects_{0};
  int consecutive_misses_{0};
  bool filtered_valid_{false};
  bool has_last_detection_{false};
  double last_u_{0.0};
  double last_v_{0.0};
  bool has_last_theta_{false};
  double last_theta_x_{0.0};
  double last_theta_y_{0.0};

  bool has_caminfo_{false};
  double fx_{0}, fy_{0}, cx_{0}, cy_{0};

  bool has_nav_depth_{false};
  bool has_nav_heading_{false};
  bool has_dock_depth_{false};
  double nav_depth_{0.0};
  double nav_heading_deg_{0.0};
  double dock_depth_{0.0};

  bool has_last_next_xy_{false};
  double last_next_x_{0.0};
  double last_next_y_{0.0};

  ros::Time last_frame_time_;
  ros::Time last_timeout_publish_;
  std::string last_frame_id_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "docking_optical_vision_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  OpticalVisionNode node(nh, pnh);
  ros::spin();
  return 0;
}
