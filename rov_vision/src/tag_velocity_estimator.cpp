#include "rov_vision/tag_velocity_estimator.hpp"

namespace rov_vision {

VelocityEstimator::VelocityEstimator() : Node("velocity_estimator_node") {
  this->declare_parameter<std::string>("corners_topic", "/apriltag/corners");
  this->declare_parameter<std::string>("velocity_topic",
                                       "/rov_vision/camera_velocity");
  this->declare_parameter<std::string>("marker_topic",
                                       "/rov_vision/velocity_marker");
  this->declare_parameter<double>("min_dt", 1e-3);
  this->declare_parameter<double>("max_dt", 0.5);
  this->declare_parameter<double>("marker_scale", 1.0);

  const auto corners_topic = this->get_parameter("corners_topic").as_string();
  const auto velocity_topic = this->get_parameter("velocity_topic").as_string();
  const auto marker_topic = this->get_parameter("marker_topic").as_string();
  min_dt_ = this->get_parameter("min_dt").as_double();
  max_dt_ = this->get_parameter("max_dt").as_double();
  marker_scale_ = this->get_parameter("marker_scale").as_double();

  corners_sub_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
      corners_topic, rclcpp::SensorDataQoS(),
      std::bind(&VelocityEstimator::cornersCallback, this,
                std::placeholders::_1));

  velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      velocity_topic, 10);
  marker_pub_ =
      this->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10);

  RCLCPP_INFO(this->get_logger(),
              "VelocityEstimator started\n"
              "  corners in  <- %s\n"
              "  twist out   -> %s\n"
              "  marker out  -> %s",
              corners_topic.c_str(), velocity_topic.c_str(),
              marker_topic.c_str());
}

Eigen::Matrix<double, 2, 6> VelocityEstimator::interactionMatrix(
    double x, double y, double Z) const {
  Eigen::Matrix<double, 2, 6> L;
  // clang-format off
  L << -1.0 / Z,      0.0,     x / Z,       x * y,   -(1.0 + x * x),   y,
            0.0,  -1.0 / Z,     y / Z,  1.0 + y * y,       -x * y,    -x;
  // clang-format on
  return L;
}

void VelocityEstimator::cornersCallback(
    const geometry_msgs::msg::PolygonStamped::ConstSharedPtr &msg) {
  if (msg->polygon.points.size() != 4) {
    return;
  }

  std::array<Eigen::Vector3d, 4> pts;
  for (size_t i = 0; i < 4; ++i) {
    pts[i] = Eigen::Vector3d(msg->polygon.points[i].x, msg->polygon.points[i].y,
                             msg->polygon.points[i].z);
    if (pts[i].z() < 1e-3) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Corner %zu has near-zero depth, skipping frame", i);
      return;
    }
  }

  const rclcpp::Time stamp(msg->header.stamp);

  if (!prev_frame_.has_value()) {
    prev_frame_ = PrevFrame{pts, stamp};
    return;
  }

  const double dt = (stamp - prev_frame_->stamp).seconds();
  if (dt < min_dt_ || dt > max_dt_) {
    prev_frame_ = PrevFrame{pts, stamp};
    return;
  }

  Eigen::Matrix<double, 8, 6> L_stack;
  Eigen::Matrix<double, 8, 1> xdot_stack;
  Eigen::Vector3d centroid_curr = Eigen::Vector3d::Zero();

  for (size_t i = 0; i < 4; ++i) {
    const double x_curr = pts[i].x() / pts[i].z();
    const double y_curr = pts[i].y() / pts[i].z();
    const double x_prev =
        prev_frame_->points[i].x() / prev_frame_->points[i].z();
    const double y_prev =
        prev_frame_->points[i].y() / prev_frame_->points[i].z();

    const double xdot = (x_curr - x_prev) / dt;
    const double ydot = (y_curr - y_prev) / dt;

    L_stack.block<2, 6>(2 * i, 0) =
        interactionMatrix(x_curr, y_curr, pts[i].z());
    xdot_stack(2 * i) = xdot;
    xdot_stack(2 * i + 1) = ydot;

    centroid_curr += pts[i];
  }
  centroid_curr /= 4.0;

  const Eigen::Matrix<double, 6, 1> v_camera =
      L_stack.completeOrthogonalDecomposition().solve(xdot_stack);

  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header = msg->header;
  twist_msg.twist.linear.x = v_camera(0);
  twist_msg.twist.linear.y = v_camera(1);
  twist_msg.twist.linear.z = v_camera(2);
  twist_msg.twist.angular.x = v_camera(3);
  twist_msg.twist.angular.y = v_camera(4);
  twist_msg.twist.angular.z = v_camera(5);
  velocity_pub_->publish(twist_msg);

  RCLCPP_INFO(this->get_logger(),
              "v=[%.3f %.3f %.3f] m/s  w=[%.3f %.3f %.3f] rad/s  (dt=%.4fs)",
              v_camera(0), v_camera(1), v_camera(2), v_camera(3), v_camera(4),
              v_camera(5), dt);

  geometry_msgs::msg::Point origin;
  origin.x = centroid_curr.x();
  origin.y = centroid_curr.y();
  origin.z = centroid_curr.z();
  publishMarker(origin, v_camera.head<3>(), msg->header);

  prev_frame_ = PrevFrame{pts, stamp};
}

void VelocityEstimator::publishMarker(const geometry_msgs::msg::Point &origin,
                                      const Eigen::Vector3d &v_lin,
                                      const std_msgs::msg::Header &header) {
  visualization_msgs::msg::Marker marker;
  marker.header = header;
  marker.ns = "camera_velocity";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point tip;
  tip.x = origin.x + v_lin.x() * marker_scale_;
  tip.y = origin.y + v_lin.y() * marker_scale_;
  tip.z = origin.z + v_lin.z() * marker_scale_;

  marker.points.push_back(origin);
  marker.points.push_back(tip);

  marker.scale.x = 0.02;  // shaft diameter
  marker.scale.y = 0.04;  // head diameter
  marker.scale.z = 0.0;   // auto head length

  marker.color.r = 1.0f;
  marker.color.g = 0.2f;
  marker.color.b = 0.2f;
  marker.color.a = 1.0f;

  marker.lifetime = rclcpp::Duration::from_seconds(0.3);

  marker_pub_->publish(marker);
}

}  // namespace rov_vision

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_vision::VelocityEstimator>());
  rclcpp::shutdown();
  return 0;
}
