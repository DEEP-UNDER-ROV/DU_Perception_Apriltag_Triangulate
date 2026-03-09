#include "rov_vision/stereo_triangulator.hpp"
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

namespace rov_vision {

StereoTriangulator::StereoTriangulator() : Node("stereo_triangulate_node") {
  this->declare_parameter<std::string>("detection1_topic", "/detection1");
  this->declare_parameter<std::string>("detection2_topic", "/detection2");
  this->declare_parameter<std::string>("left_camera_info_topic",
                                       "/rov/left/camera_info");
  this->declare_parameter<std::string>("right_camera_info_topic",
                                       "/rov/right/camera_info");
  this->declare_parameter<std::string>("corners_topic", "/apriltag/corners");

  const auto det1_topic = this->get_parameter("detection1_topic").as_string();
  const auto det2_topic = this->get_parameter("detection2_topic").as_string();
  const auto left_info_topic =
      this->get_parameter("left_camera_info_topic").as_string();
  const auto right_info_topic =
      this->get_parameter("right_camera_info_topic").as_string();
  const auto corners_topic = this->get_parameter("corners_topic").as_string();

  left_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      left_info_topic, 10, [this](const sensor_msgs::msg::CameraInfo &msg) {
        if (left_info_ready_)
          return;
        fx_ = static_cast<float>(msg.k[0]);
        fy_ = static_cast<float>(msg.k[4]);
        cx_ = static_cast<float>(msg.k[2]);
        cy_ = static_cast<float>(msg.k[5]);
        left_info_ready_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Left CameraInfo latched:\n"
                    "  fx=%.3f  fy=%.3f  cx=%.3f  cy=%.3f",
                    fx_, fy_, cx_, cy_);
      });

  right_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      right_info_topic, 10, [this](const sensor_msgs::msg::CameraInfo &msg) {
        if (right_info_ready_)
          return;

        // P[3]  = -fx * tx  →  bf = -P[3],  tx = -P[3]/fx
        // P[7]  = -fy * ty  →  ty = -P[7]/fy
        bf_ = static_cast<float>(-msg.p[3]);
        tx_ = static_cast<float>(-msg.p[3] / msg.p[0]); // p[0] = fx in P
        ty_ = static_cast<float>(-msg.p[7] / msg.p[5]); // p[5] = fy in P

        // R[9] rotation matrix (row-major)
        R_ = msg.r;

        right_info_ready_ = true;
        RCLCPP_INFO(this->get_logger(),
                    "Right CameraInfo latched:\n"
                    "  bf=%.6f  tx=%.5fm  ty=%.5fm\n"
                    "  R=[%.3f %.3f %.3f | %.3f %.3f %.3f | %.3f %.3f %.3f]",
                    bf_, tx_, ty_, R_[0], R_[1], R_[2], R_[3], R_[4], R_[5],
                    R_[6], R_[7], R_[8]);
      });

  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  detection1_sub_.subscribe(this, det1_topic);
  detection2_sub_.subscribe(this, det2_topic);

  sync_ = std::make_shared<Synchronizer>(SyncPolicy(10), detection1_sub_,
                                         detection2_sub_);
  sync_->registerCallback(
      std::bind(&StereoTriangulator::triangulate, this, _1, _2));

  corners_pub_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
      corners_topic, qos);

  RCLCPP_INFO(this->get_logger(),
              "StereoTriangulator started — waiting for CameraInfo on:\n"
              "  left  → %s\n  right → %s",
              left_info_topic.c_str(), right_info_topic.c_str());
}

Eigen::Vector3d StereoTriangulator::triangulatePoint(float u_left, float v_left,
                                                     float u_right) {

  // disparity = u_left - u_right
  // Z = bf / disparity   (bf = fx * baseline)
  // X = (u_left - cx) * Z / fx
  // Y = (v_left - cy) * Z / fy
  float disparity = u_left - u_right;
  if (std::abs(disparity) < 1e-5f)
    return Eigen::Vector3d::Zero();

  float Z = bf_ / disparity;
  float X = (u_left - cx_) * Z / fx_;
  float Y = (v_left - cy_) * Z / fy_;
  return Eigen::Vector3d(X, Y, Z);
}

void StereoTriangulator::triangulate(
    const DetectionArray::ConstSharedPtr &left_msg,
    const DetectionArray::ConstSharedPtr &right_msg) {

  if (!left_info_ready_ || !right_info_ready_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Waiting for CameraInfo — left=%s  right=%s",
                         left_info_ready_ ? "OK" : "...",
                         right_info_ready_ ? "OK" : "...");
    return;
  }

  if (left_msg->detections.empty() || right_msg->detections.empty())
    return;

  for (const auto &left_det : left_msg->detections) {
    for (const auto &right_det : right_msg->detections) {
      if (left_det.id != right_det.id)
        continue;
      if (left_det.corners.size() != 4 || right_det.corners.size() != 4)
        continue;

      auto polygon_msg = geometry_msgs::msg::PolygonStamped();
      polygon_msg.header = left_msg->header;
      bool valid = true;

      // corners[0]=top-left [1]=top-right [2]=bottom-right [3]=bottom-left
      for (size_t i = 0; i < 4; ++i) {
        auto point =
            triangulatePoint(left_det.corners[i].x, left_det.corners[i].y,
                             right_det.corners[i].x);

        if (point.isZero()) {
          RCLCPP_WARN(this->get_logger(),
                      "Tag ID %d corner %zu: near-zero disparity, skipping",
                      left_det.id, i);
          valid = false;
          break;
        }

        geometry_msgs::msg::Point32 pt;
        pt.x = static_cast<float>(point.x());
        pt.y = static_cast<float>(point.y());
        pt.z = static_cast<float>(point.z());
        polygon_msg.polygon.points.push_back(pt);
      }

      if (valid && polygon_msg.polygon.points.size() == 4) {
        corners_pub_->publish(polygon_msg);
        RCLCPP_INFO(this->get_logger(),
                    "Tag ID %d — corners published (Z=%.3fm)", left_det.id,
                    polygon_msg.polygon.points[0].z);
      }
    }
  }
}

} // namespace rov_vision

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rov_vision::StereoTriangulator>());
  rclcpp::shutdown();
  return 0;
}
