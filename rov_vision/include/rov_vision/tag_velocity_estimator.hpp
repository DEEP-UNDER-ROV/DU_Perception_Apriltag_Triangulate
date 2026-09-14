#pragma once

#include <Eigen/Dense>
#include <array>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace rov_vision {

class VelocityEstimator : public rclcpp::Node {
 public:
  VelocityEstimator();

 private:
  void cornersCallback(
      const geometry_msgs::msg::PolygonStamped::ConstSharedPtr &msg);

  Eigen::Matrix<double, 2, 6> interactionMatrix(double x, double y,
                                                double Z) const;

  void publishMarker(const geometry_msgs::msg::Point &origin,
                     const Eigen::Vector3d &v_lin,
                     const std_msgs::msg::Header &header);

  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr
      corners_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  struct PrevFrame {
    std::array<Eigen::Vector3d, 4> points;  // camera-frame 3D corners
    rclcpp::Time stamp;
  };
  std::optional<PrevFrame> prev_frame_;

  double min_dt_{1e-3};
  double max_dt_{0.5};
  double marker_scale_{1.0};
};

}  // namespace rov_vision
