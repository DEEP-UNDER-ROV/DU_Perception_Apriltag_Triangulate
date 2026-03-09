#include <array>
#include <camera_info_manager/camera_info_manager.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vector>

class CameraInfoPublisher : public rclcpp::Node {
public:
  CameraInfoPublisher() : Node("camera_info_publisher") {
    this->declare_parameter("camera_info_url", "");
    this->declare_parameter("camera_name", "camera");
    this->declare_parameter("topic", "camera_info");
    this->declare_parameter("frame_id", "camera_optical_frame");
    this->declare_parameter("parent_frame_id", "");
    this->declare_parameter("sync_topic", "");
    this->declare_parameter("baseline", 0.0);
    this->declare_parameter("bf", 0.0);
    this->declare_parameter("extrinsic",
                            std::vector<double>{1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                                0.0, 0.0, 0.0, 1.0});

    const auto url = this->get_parameter("camera_info_url").as_string();
    const auto name = this->get_parameter("camera_name").as_string();
    const auto topic = this->get_parameter("topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    const auto parent_frame =
        this->get_parameter("parent_frame_id").as_string();
    const auto sync_topic = this->get_parameter("sync_topic").as_string();
    const auto baseline = this->get_parameter("baseline").as_double();
    const auto bf = this->get_parameter("bf").as_double();
    const auto extrinsic = this->get_parameter("extrinsic").as_double_array();

    if (extrinsic.size() < 16) {
      RCLCPP_ERROR(this->get_logger(), "Extrinsic must be 16 values (4x4)");
      return;
    }

    std::array<double, 9> R;
    R[0] = extrinsic[0];
    R[1] = extrinsic[1];
    R[2] = extrinsic[2];
    R[3] = extrinsic[4];
    R[4] = extrinsic[5];
    R[5] = extrinsic[6];
    R[6] = extrinsic[8];
    R[7] = extrinsic[9];
    R[8] = extrinsic[10];

    const double tx = extrinsic[3];
    const double ty = extrinsic[7];
    const double tz = extrinsic[11];

    RCLCPP_INFO(this->get_logger(), "Loading calibration from: %s",
                url.c_str());
    manager_ =
        std::make_shared<camera_info_manager::CameraInfoManager>(this, name);
    manager_->loadCameraInfo(url);

    if (!manager_->isCalibrated()) {
      RCLCPP_WARN(this->get_logger(), "Not calibrated, check: %s", url.c_str());
    }

    auto info = manager_->getCameraInfo();
    const double fx = info.k[0];
    const double fy = info.k[4];
    const double cx = info.k[2];
    const double cy = info.k[5];

    info.r = R;
    if (tx != 0.0 || ty != 0.0) {
      info.p[3] = -fx * tx;
      info.p[7] = -fy * ty;
    }

    calibrated_info_ = info;

    RCLCPP_INFO(this->get_logger(),
                "Calibration loaded:\n"
                "  Intrinsics: fx=%.3f  fy=%.3f  cx=%.3f  cy=%.3f\n"
                "  Extrinsic t=[%.5f, %.5f, %.5f]\n"
                "  P[3]=%.6f (Tx)  P[7]=%.6f (Ty)  bf=%.6f  baseline=%.5fm",
                fx, fy, cx, cy, tx, ty, tz, info.p[3], info.p[7], bf, baseline);

    pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(topic, 10);

    // Subscribe to corrected camera_info just for timestamp sync
    if (!sync_topic.empty()) {
      RCLCPP_INFO(this->get_logger(), "Syncing timestamps from: %s",
                  sync_topic.c_str());
      sync_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
          sync_topic, 10, [this](const sensor_msgs::msg::CameraInfo &msg) {
            calibrated_info_.header.stamp = msg.header.stamp;
            calibrated_info_.header.frame_id = frame_id_;
            pub_->publish(calibrated_info_);
          });
    } else {
      RCLCPP_WARN(this->get_logger(), "No sync_topic set — publishing at fixed "
                                      "30Hz (may desync with images)");
      timer_ = this->create_wall_timer(std::chrono::milliseconds(33), [this]() {
        calibrated_info_.header.stamp = this->get_clock()->now();
        calibrated_info_.header.frame_id = frame_id_;
        pub_->publish(calibrated_info_);
      });
    }

    if (!parent_frame.empty()) {
      publishStaticTF(parent_frame, frame_id_, tx, ty, tz, R);
    }

    RCLCPP_INFO(this->get_logger(), "Publishing on: %s [frame_id: %s]",
                topic.c_str(), frame_id_.c_str());
  }

private:
  void publishStaticTF(const std::string &parent_frame,
                       const std::string &child_frame, double tx, double ty,
                       double tz, const std::array<double, 9> &R) {

    tf2::Matrix3x3 rot(R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8]);
    tf2::Quaternion q;
    rot.getRotation(q);
    q.normalize();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = child_frame;
    tf_msg.transform.translation.x = tx;
    tf_msg.transform.translation.y = ty;
    tf_msg.transform.translation.z = tz;
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_broadcaster_->sendTransform(tf_msg);

    RCLCPP_INFO(this->get_logger(), "Static TF: %s → %s  t=[%.5f, %.5f, %.5f]",
                parent_frame.c_str(), child_frame.c_str(), tx, ty, tz);
  }

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sync_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> manager_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  sensor_msgs::msg::CameraInfo calibrated_info_;
  std::string frame_id_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}
