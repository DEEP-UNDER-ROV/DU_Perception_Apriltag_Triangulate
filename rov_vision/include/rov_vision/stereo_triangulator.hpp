#pragma once
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <eigen3/Eigen/Core>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rov_vision {

using DetectionArray = apriltag_msgs::msg::AprilTagDetectionArray;
using SyncPolicy =
    message_filters::sync_policies::ApproximateTime<DetectionArray,
                                                    DetectionArray>;
using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

class StereoTriangulator : public rclcpp::Node {
public:
  StereoTriangulator();
  ~StereoTriangulator() = default;

private:
  void triangulate(const DetectionArray::ConstSharedPtr &left_msg,
                   const DetectionArray::ConstSharedPtr &right_msg);

  Eigen::Vector3d triangulatePoint(float u_left, float v_left, float u_right);

  message_filters::Subscriber<DetectionArray> detection1_sub_;
  message_filters::Subscriber<DetectionArray> detection2_sub_;
  std::shared_ptr<Synchronizer> sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr corners_pub_;

  // Intrinsics from left CameraInfo
  float fx_{0}, fy_{0}, cx_{0}, cy_{0};

  // Extrinsics from right CameraInfo
  float bf_{0}; // -P[3] = fx * tx  (for depth)
  float tx_{0}; // baseline in meters  (= bf/fx)
  float ty_{0}; // vertical offset     (-P[7]/fy)

  // Rotation matrix from right CameraInfo.r (row-major 3x3)
  std::array<double, 9> R_{1, 0, 0, 0, 1, 0, 0, 0, 1};

  bool left_info_ready_{false};
  bool right_info_ready_{false};
};

} // namespace rov_vision
