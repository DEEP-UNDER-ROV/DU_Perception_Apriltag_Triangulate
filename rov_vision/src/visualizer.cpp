// visualizer: stereo (infra1/infra2) depth estimation with a colour-coded
// legend, selectable colour / depth / alternating stream, sent to a GCS through
// GStreamer.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rov_vision/srv/set_stream_mode.hpp"
#include "sensor_msgs/msg/image.hpp"

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

using Image = sensor_msgs::msg::Image;
using SetStreamMode = rov_vision::srv::SetStreamMode;
using SteadyClock = std::chrono::steady_clock;

namespace {

enum class Mode { COLOR = 0, DEPTH = 1, ALTERNATE = 2 };

const char* mode_to_string(Mode m) {
  switch (m) {
    case Mode::COLOR:
      return "color";
    case Mode::DEPTH:
      return "depth";
    case Mode::ALTERNATE:
      return "alternate";
  }
  return "color";
}

bool parse_mode(const std::string& s, Mode& out) {
  if (s == "color") {
    out = Mode::COLOR;
    return true;
  }
  if (s == "depth") {
    out = Mode::DEPTH;
    return true;
  }
  if (s == "alternate") {
    out = Mode::ALTERNATE;
    return true;
  }
  return false;
}

struct Intrinsics {
  double fx{1}, fy{1}, cx{0}, cy{0}, k1{0}, k2{0}, p1{0}, p2{0};
  cv::Mat K() const {
    return (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  }
  cv::Mat D() const { return (cv::Mat_<double>(1, 4) << k1, k2, p1, p2); }
  bool has_distortion() const {
    return k1 != 0 || k2 != 0 || p1 != 0 || p2 != 0;
  }
};

std::string fmt_depth(double m) {
  char b[32];
  if (m < 1.0) {
    std::snprintf(b, sizeof b, "%.0f cm", m * 100.0);
  } else {
    std::snprintf(b, sizeof b, "%.1f m", m);
  }
  return b;
}

void draw_label(cv::Mat& img, const std::string& text, cv::Point org) {
  cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(0, 0, 0), 3, cv::LINE_AA);
  cv::putText(img, text, org, cv::FONT_HERSHEY_SIMPLEX, 0.6,
              cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
}

struct IrFrame {
  cv::Mat img;
  double t{0};
  std_msgs::msg::Header header;
  bool valid{false};
};

struct TimedFrame {
  cv::Mat img;
  SteadyClock::time_point arrived;
};

}  // namespace

class Visualizer : public rclcpp::Node {
 public:
  Visualizer() : Node("visualizer") {
    declare_all_parameters();
    setup_stereo();
    build_legend();

    // Callback groups: stereo processing must not block colour reception or the
    // output timer.
    cb_stereo_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_color_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_out_ =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(qos_depth_))};
    if (qos_reliable_) {
      qos.reliable();
    } else {
      qos.best_effort();
    }

    rclcpp::SubscriptionOptions o_stereo, o_color;
    o_stereo.callback_group = cb_stereo_;
    o_color.callback_group = cb_color_;

    sub_color_ = create_subscription<Image>(
        topic_color_, qos, [this](Image::ConstSharedPtr m) { on_color(m); },
        o_color);
    sub_ir1_ = create_subscription<Image>(
        topic_ir1_, qos, [this](Image::ConstSharedPtr m) { on_infra(true, m); },
        o_stereo);
    sub_ir2_ = create_subscription<Image>(
        topic_ir2_, qos,
        [this](Image::ConstSharedPtr m) { on_infra(false, m); }, o_stereo);

    if (publish_ros_image_) {
      pub_out_ = create_publisher<Image>(topic_out_, 1);
    }
    if (publish_depth_) {
      pub_depth_ = create_publisher<Image>(topic_depth_, 1);
    }

    srv_mode_ = create_service<SetStreamMode>(
        "~/set_stream_mode",
        [this](const SetStreamMode::Request::SharedPtr req,
               SetStreamMode::Response::SharedPtr res) {
          on_set_mode(req, res);
        },
        rclcpp::ServicesQoS(), cb_out_);

    param_cb_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& ps) {
          return on_params(ps);
        });

    if (gcs_enabled_) {
      open_writer();
    }

    timer_ = create_wall_timer(
        std::chrono::milliseconds(
            event_driven_ ? 1000
                          : static_cast<int>(std::lround(1000.0 / fps_))),
        [this]() { on_timer(); }, cb_out_);

    RCLCPP_INFO(get_logger(),
                "visualizer up. mode=%s, color=%s, infra1=%s, infra2=%s, "
                "out=%dx%d@%.1f",
                mode_to_string(mode_.load()), topic_color_.c_str(),
                topic_ir1_.c_str(), topic_ir2_.c_str(), out_size_.width,
                out_size_.height, fps_);
  }

 private:
  // ------------------------------------------------------------------
  // parameters
  template <typename T>
  T P(const std::string& name, const T& def) {
    return declare_parameter<T>(name, def);
  }

  void declare_all_parameters() {
    topic_color_ =
        P<std::string>("topics.color", "/camera/camera/color/image_raw");
    topic_ir1_ =
        P<std::string>("topics.infra1", "/camera/camera/infra1/image_rect_raw");
    topic_ir2_ =
        P<std::string>("topics.infra2", "/camera/camera/infra2/image_rect_raw");
    topic_out_ = P<std::string>("topics.output", "/visualizer/image");
    topic_depth_ = P<std::string>("topics.depth", "/visualizer/depth");
    qos_reliable_ = P<bool>("qos.reliable", false);
    qos_depth_ = std::max<int>(1, P<int>("qos.depth", 2));

    cam_w_ = P<int>("camera.width", 848);
    cam_h_ = P<int>("camera.height", 480);
    cam_l_.fx = P<double>("camera.fx", 430.793086);
    cam_l_.fy = P<double>("camera.fy", 430.793086);
    cam_l_.cx = P<double>("camera.cx", 414.830803);
    cam_l_.cy = P<double>("camera.cy", 244.104872);
    cam_l_.k1 = P<double>("camera.k1", 0.0);
    cam_l_.k2 = P<double>("camera.k2", 0.0);
    cam_l_.p1 = P<double>("camera.p1", 0.0);
    cam_l_.p2 = P<double>("camera.p2", 0.0);
    baseline_ = P<double>("camera.baseline", 0.09638894907585159);
    const double bf = P<double>("camera.bf", 41.523693);
    cam_r_.fx = P<double>("camera2.fx", cam_l_.fx);
    cam_r_.fy = P<double>("camera2.fy", cam_l_.fy);
    cam_r_.cx = P<double>("camera2.cx", cam_l_.cx);
    cam_r_.cy = P<double>("camera2.cy", cam_l_.cy);
    cam_r_.k1 = P<double>("camera2.k1", 0.0);
    cam_r_.k2 = P<double>("camera2.k2", 0.0);
    cam_r_.p1 = P<double>("camera2.p1", 0.0);
    cam_r_.p2 = P<double>("camera2.p2", 0.0);
    if (baseline_ <= 0.0 && bf > 0.0) {
      baseline_ = bf / cam_l_.fx;
    }
    if (baseline_ <= 0.0) {
      throw std::runtime_error("camera.baseline (or camera.bf) must be > 0");
    }
    if (cam_w_ <= 0 || cam_h_ <= 0) {
      throw std::runtime_error("camera.width/height must be > 0");
    }

    proc_scale_ =
        std::clamp(P<double>("stereo.processing_scale", 0.5), 0.1, 1.0);
    num_disp_ = P<int>("stereo.num_disparities", 208);
    min_disp_ = P<int>("stereo.min_disparity", 0);
    block_size_ = P<int>("stereo.block_size", 7);
    sgbm_mode_ = P<std::string>("stereo.mode", "sgbm");
    p1_scale_ = P<int>("stereo.p1_scale", 8);
    p2_scale_ = P<int>("stereo.p2_scale", 32);
    uniq_ = P<int>("stereo.uniqueness_ratio", 10);
    speckle_win_ = P<int>("stereo.speckle_window_size", 100);
    speckle_rng_ = P<int>("stereo.speckle_range", 2);
    disp12_ = P<int>("stereo.disp12_max_diff", 1);
    prefilter_cap_ = P<int>("stereo.pre_filter_cap", 31);
    median_k_ = P<int>("stereo.median_ksize", 3);
    slop_ = P<double>("stereo.sync_slop_sec", 0.02);
    pad_left_ = P<bool>("stereo.pad_left", true);
    crop_strip_ = P<bool>("stereo.crop_blank_strip", false);

    min_m_ = P<double>("depth.min_m", 0.10);
    max_m_ = P<double>("depth.max_m", 3.0);
    if (min_m_ < 0.0 || max_m_ <= min_m_) {
      throw std::runtime_error("need 0 <= depth.min_m < depth.max_m");
    }
    const std::string cmap = P<std::string>("depth.colormap", "jet");
    cmap_ = (cmap == "turbo") ? cv::COLORMAP_TURBO : cv::COLORMAP_JET;

    legend_enabled_ = P<bool>("legend.enabled", true);
    legend_ticks_ = std::max(2, P<int>("legend.ticks", 6));
    legend_w_ = std::max(60, P<int>("legend.width", 84));

    const std::string mode_s = P<std::string>("stream.mode", "color");
    Mode m = Mode::COLOR;
    if (!parse_mode(mode_s, m)) {
      RCLCPP_WARN(get_logger(), "unknown stream.mode '%s', using color",
                  mode_s.c_str());
    }
    mode_ = m;
    alt_color_sec_ = P<double>("stream.alternate_color_sec", 5.0);
    alt_depth_sec_ = P<double>("stream.alternate_depth_sec", 5.0);
    event_driven_ = P<bool>("stream.event_driven", true);
    fps_ = std::max(1.0, P<double>("stream.fps", 15.0));
    out_size_ = cv::Size(std::max(64, P<int>("stream.output_width", 848)),
                         std::max(64, P<int>("stream.output_height", 480)));
    stale_sec_ = P<double>("stream.stale_timeout_sec", 1.0);
    overlay_mode_ = P<bool>("stream.overlay_mode_text", true);
    publish_ros_image_ = P<bool>("stream.publish_ros_image", false);
    publish_depth_ = P<bool>("stream.publish_depth", false);

    gcs_enabled_ = P<bool>("gcs.enabled", true);
    gcs_ip_ = P<std::string>("gcs.ip", "127.0.0.1");
    gcs_port_ = P<int>("gcs.port", 5600);
    gcs_encoder_ = P<std::string>("gcs.encoder", "x264");
    gcs_bitrate_ = P<int>("gcs.bitrate_kbps", 2000);
    gcs_pipeline_ = P<std::string>("gcs.pipeline", "");

    need_depth_ = (mode_.load() != Mode::COLOR);
    alt_start_ = now_steady_sec();
  }

  // Only the stream mode and the alternate durations can change while running.
  rcl_interfaces::msg::SetParametersResult on_params(
      const std::vector<rclcpp::Parameter>& ps) {
    rcl_interfaces::msg::SetParametersResult res;
    res.successful = true;

    for (const auto& p : ps) {
      const auto& n = p.get_name();
      if (n == "stream.mode") {
        Mode m;
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
            !parse_mode(p.as_string(), m)) {
          res.successful = false;
          res.reason = "stream.mode must be one of: color, depth, alternate";
          return res;
        }
      } else if (n == "stream.alternate_color_sec" ||
                 n == "stream.alternate_depth_sec") {
        if (p.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
            p.as_double() <= 0.0) {
          res.successful = false;
          res.reason = n + " must be a double > 0";
          return res;
        }
      } else if (n != "use_sim_time") {
        res.successful = false;
        res.reason = "'" + n + "' can only be set at startup";
        return res;
      }
    }

    // Apply durations first so a mode change to "alternate" starts with the new
    // timings.
    for (const auto& p : ps) {
      if (p.get_name() == "stream.alternate_color_sec") {
        alt_color_sec_ = p.as_double();
      }
      if (p.get_name() == "stream.alternate_depth_sec") {
        alt_depth_sec_ = p.as_double();
      }
    }
    for (const auto& p : ps) {
      if (p.get_name() == "stream.mode") {
        Mode m = Mode::COLOR;
        parse_mode(p.as_string(), m);
        alt_start_ = now_steady_sec();
        need_depth_ = (m != Mode::COLOR);
        mode_ = m;
        RCLCPP_INFO(get_logger(), "stream mode -> %s", mode_to_string(m));
      }
    }
    return res;
  }

  void on_set_mode(const SetStreamMode::Request::SharedPtr req,
                   SetStreamMode::Response::SharedPtr res) {
    Mode m;
    switch (req->mode) {
      case SetStreamMode::Request::COLOR:
        m = Mode::COLOR;
        break;
      case SetStreamMode::Request::DEPTH:
        m = Mode::DEPTH;
        break;
      case SetStreamMode::Request::ALTERNATE:
        m = Mode::ALTERNATE;
        break;
      default:
        res->success = false;
        res->message = "invalid mode " + std::to_string(req->mode) +
                       " (0=color 1=depth 2=alternate)";
        return;
    }
    std::vector<rclcpp::Parameter> ps;
    if (req->color_duration > 0.0f) {
      ps.emplace_back("stream.alternate_color_sec",
                      static_cast<double>(req->color_duration));
    }
    if (req->depth_duration > 0.0f) {
      ps.emplace_back("stream.alternate_depth_sec",
                      static_cast<double>(req->depth_duration));
    }
    ps.emplace_back("stream.mode", std::string(mode_to_string(m)));

    const auto r = set_parameters_atomically(ps);
    res->success = r.successful;
    res->message = r.successful
                       ? std::string("mode set to ") + mode_to_string(m)
                       : r.reason;
  }

  // ------------------------------------------------------------------ stereo
  // setup
  void setup_stereo() {
    num_disp_ = std::max(16, ((num_disp_ + 15) / 16) * 16);
    if (block_size_ % 2 == 0) {
      ++block_size_;
    }
    block_size_ = std::clamp(block_size_, 1, 21);
    if (median_k_ != 0 && median_k_ != 3 && median_k_ != 5) {
      median_k_ = 3;
    }

    int mode = cv::StereoSGBM::MODE_SGBM;
    if (sgbm_mode_ == "sgbm_3way") {
      mode = cv::StereoSGBM::MODE_SGBM_3WAY;
    } else if (sgbm_mode_ == "hh") {
      mode = cv::StereoSGBM::MODE_HH;
    } else if (sgbm_mode_ == "hh4") {
      mode = cv::StereoSGBM::MODE_HH4;
    }

    const int cn = 1;
    matcher_ = cv::StereoSGBM::create(
        min_disp_, num_disp_, block_size_,
        p1_scale_ * cn * block_size_ * block_size_,
        p2_scale_ * cn * block_size_ * block_size_, disp12_, prefilter_cap_,
        uniq_, speckle_win_, speckle_rng_, mode);

    const double min_depth_meas =
        cam_l_.fx * proc_scale_ * baseline_ / num_disp_;
    RCLCPP_INFO(
        get_logger(),
        "SGBM: scale=%.2f, numDisp=%d, block=%d -> nearest measurable ~%.0f cm",
        proc_scale_, num_disp_, block_size_, min_depth_meas * 100.0);
    if (min_depth_meas > min_m_) {
      RCLCPP_WARN(get_logger(),
                  "depth.min_m=%.2f m is closer than the matcher can see (%.2f "
                  "m). Raise "
                  "stereo.num_disparities or stereo.processing_scale.",
                  min_m_, min_depth_meas);
    }
  }

  void ensure_rectify_maps(const cv::Size& sz) {
    if (sz == maps_size_) {
      return;
    }
    maps_size_ = sz;
    map1_l_.release();
    map2_l_.release();
    map1_r_.release();
    map2_r_.release();

    if (sz.width != cam_w_ || sz.height != cam_h_) {
      RCLCPP_WARN(
          get_logger(),
          "image is %dx%d but camera.width/height is %dx%d: scaling intrinsics",
          sz.width, sz.height, cam_w_, cam_h_);
    }
    const double sx = sz.width / static_cast<double>(cam_w_);
    const double sy = sz.height / static_cast<double>(cam_h_);
    auto scaled = [&](Intrinsics i) {
      i.fx *= sx;
      i.cx *= sx;
      i.fy *= sy;
      i.cy *= sy;
      return i;
    };
    const Intrinsics L = scaled(cam_l_), R = scaled(cam_r_);
    // Both images are mapped to the left camera matrix so the pair stays
    // row-aligned.
    if (L.has_distortion()) {
      cv::initUndistortRectifyMap(L.K(), L.D(), cv::Mat(), L.K(), sz, CV_16SC2,
                                  map1_l_, map2_l_);
    }
    if (R.has_distortion()) {
      cv::initUndistortRectifyMap(R.K(), R.D(), cv::Mat(), L.K(), sz, CV_16SC2,
                                  map1_r_, map2_r_);
    }
  }

  // ------------------------------------------------------------------
  // callbacks
  void on_color(const Image::ConstSharedPtr& msg) {
    cv::Mat bgr;
    try {
      bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                            "colour convert: %s", e.what());
      return;
    }
    cv::Mat out;
    if (bgr.size() != out_size_) {
      cv::resize(bgr, out, out_size_, 0, 0, cv::INTER_AREA);
    } else {
      out = bgr.clone();
    }
    {
      std::lock_guard<std::mutex> lk(color_mtx_);
      color_ = {out, SteadyClock::now()};
    }
    if (event_driven_ && !selected_depth()) {
      emit_frame();
    }
  }

  void on_infra(bool left, const Image::ConstSharedPtr& msg) {
    if (!need_depth_) {
      return;
    }  // colour-only mode: don't burn CPU on stereo matching

    cv::Mat img;
    try {
      img = cv_bridge::toCvShare(msg, "mono8")->image.clone();
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                            "infra convert: %s", e.what());
      return;
    }

    IrFrame l, r;
    bool ready = false;
    {
      std::lock_guard<std::mutex> lk(ir_mtx_);
      IrFrame& slot = left ? ir_l_ : ir_r_;
      slot.img = img;
      slot.t = rclcpp::Time(msg->header.stamp).seconds();
      slot.header = msg->header;
      slot.valid = true;
      if (ir_l_.valid && ir_r_.valid) {
        const double dt = ir_l_.t - ir_r_.t;
        if (std::abs(dt) <= slop_) {
          l = ir_l_;
          r = ir_r_;
          ir_l_.valid = ir_r_.valid = false;
          ready = true;
        } else if (dt < 0) {
          ir_l_.valid = false;  // left is older, drop it
        } else {
          ir_r_.valid = false;
        }
      }
    }
    if (ready) {
      process_stereo(l, r);
    }
  }

  void process_stereo(const IrFrame& l, const IrFrame& r) {
    if (l.img.size() != r.img.size()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000,
                            "infra1/infra2 sizes differ");
      return;
    }
    ensure_rectify_maps(l.img.size());

    cv::Mat left = l.img, right = r.img, tmp;
    if (!map1_l_.empty()) {
      cv::remap(left, tmp, map1_l_, map2_l_, cv::INTER_LINEAR);
      left = tmp;
    }
    if (!map1_r_.empty()) {
      cv::remap(right, tmp, map1_r_, map2_r_, cv::INTER_LINEAR);
      right = tmp;
    }

    if (proc_scale_ < 0.999) {
      cv::resize(left, left, cv::Size(), proc_scale_, proc_scale_,
                 cv::INTER_AREA);
      cv::resize(right, right, cv::Size(), proc_scale_, proc_scale_,
                 cv::INTER_AREA);
    }

    const int proc_cols = left.cols;

    // SGBM cannot match the leftmost num_disp columns (no partner in the right
    // image). Padding both images on the left lets it produce values there
    // (less reliable).
    const int pad = pad_left_ ? num_disp_ : 0;
    if (pad > 0) {
      cv::copyMakeBorder(left, tmp, 0, 0, pad, 0, cv::BORDER_REPLICATE);
      left = tmp;
      cv::Mat tmp2;
      cv::copyMakeBorder(right, tmp2, 0, 0, pad, 0, cv::BORDER_REPLICATE);
      right = tmp2;
    }

    cv::Mat disp16, disp;
    matcher_->compute(left, right, disp16);
    if (pad > 0) {
      disp16 = disp16(cv::Rect(pad, 0, disp16.cols - pad, disp16.rows)).clone();
    }
    disp16.convertTo(disp, CV_32F, 1.0 / 16.0);

    // Z = f * B / d   (f in pixels of the processed image)
    const double f = cam_l_.fx * (static_cast<double>(proc_cols) / cam_w_);
    const float fb = static_cast<float>(f * baseline_);
    cv::Mat depth(disp.size(), CV_32F);
    for (int y = 0; y < disp.rows; ++y) {
      const float* pd = disp.ptr<float>(y);
      float* pz = depth.ptr<float>(y);
      for (int x = 0; x < disp.cols; ++x) {
        const float d = pd[x];
        pz[x] = (d >= static_cast<float>(min_disp_) && d > 0.1f) ? fb / d : 0.f;
      }
    }
    if (median_k_ >= 3) {
      cv::medianBlur(depth, depth, median_k_);
    }

    if (pub_depth_) {
      pub_depth_->publish(
          *cv_bridge::CvImage(l.header, "32FC1", depth).toImageMsg());
    }

    // Optionally drop the blank strip and zoom (aspect preserved) so the view
    // is fully filled.
    if (crop_strip_ && !pad_left_) {
      const int c = std::min(num_disp_ + min_disp_, depth.cols / 2);
      const int w2 = depth.cols - c;
      const int h2 = std::min(
          depth.rows, static_cast<int>(std::lround(w2 * out_size_.height /
                                                   double(out_size_.width))));
      const int y0 = (depth.rows - h2) / 2;
      depth = depth(cv::Rect(c, y0, w2, h2)).clone();
    }

    cv::Mat viz = colorize(depth);
    if (viz.size() != out_size_) {
      cv::resize(viz, viz, out_size_, 0, 0, cv::INTER_LINEAR);
    }
    if (legend_enabled_) {
      apply_legend(viz);
    }

    {
      std::lock_guard<std::mutex> lk(depth_mtx_);
      depth_viz_ = {viz, SteadyClock::now()};
    }
    if (event_driven_ && selected_depth()) {
      emit_frame();
    }
  }

  // near (min_m) -> red, far (max_m) -> blue, no data -> black
  cv::Mat colorize(const cv::Mat& depth) const {
    cv::Mat gray(depth.size(), CV_8U), invalid(depth.size(), CV_8U);
    const float lo = static_cast<float>(min_m_),
                span = static_cast<float>(max_m_ - min_m_);
    for (int y = 0; y < depth.rows; ++y) {
      const float* pz = depth.ptr<float>(y);
      uchar* pg = gray.ptr<uchar>(y);
      uchar* pi = invalid.ptr<uchar>(y);
      for (int x = 0; x < depth.cols; ++x) {
        const float z = pz[x];
        if (z <= 0.f || !std::isfinite(z)) {
          pg[x] = 0;
          pi[x] = 255;
        } else {
          const float t = std::clamp((z - lo) / span, 0.f, 1.f);
          pg[x] = static_cast<uchar>(std::lround(255.f * (1.f - t)));
          pi[x] = 0;
        }
      }
    }
    cv::Mat color;
    cv::applyColorMap(gray, color, cmap_);
    color.setTo(cv::Scalar(0, 0, 0), invalid);
    return color;
  }

  // ------------------------------------------------------------------ legend
  void build_legend() {
    const int pw = std::min(legend_w_, out_size_.width / 2);
    const int ph = out_size_.height - 16;
    if (ph < 100) {
      legend_enabled_ = false;
      return;
    }
    legend_rect_ = cv::Rect(out_size_.width - pw - 8, 8, pw, ph);
    legend_layer_ = cv::Mat::zeros(ph, pw, CV_8UC3);

    cv::putText(legend_layer_, "Depth", cv::Point(8, 16),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 255), 1,
                cv::LINE_AA);

    const int bx = 8, by = 28, bw = 16, bh = ph - by - 10;
    cv::Mat ramp(bh, 1, CV_8U);
    for (int i = 0; i < bh; ++i) {
      ramp.at<uchar>(i, 0) =
          static_cast<uchar>(std::lround(255.0 * (1.0 - i / (bh - 1.0))));
    }
    cv::Mat ramp_c, bar;
    cv::applyColorMap(ramp, ramp_c, cmap_);
    cv::resize(ramp_c, bar, cv::Size(bw, bh), 0, 0, cv::INTER_NEAREST);
    bar.copyTo(legend_layer_(cv::Rect(bx, by, bw, bh)));
    cv::rectangle(legend_layer_, cv::Rect(bx, by, bw, bh),
                  cv::Scalar(255, 255, 255), 1);

    for (int i = 0; i < legend_ticks_; ++i) {
      const double frac = i / static_cast<double>(legend_ticks_ - 1);
      const int y = by + static_cast<int>(std::lround(frac * (bh - 1)));
      const double d = min_m_ + frac * (max_m_ - min_m_);
      cv::line(legend_layer_, cv::Point(bx + bw, y), cv::Point(bx + bw + 4, y),
               cv::Scalar(255, 255, 255), 1);
      cv::putText(legend_layer_, fmt_depth(d), cv::Point(bx + bw + 7, y + 4),
                  cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1,
                  cv::LINE_AA);
    }

    cv::Mat g;
    cv::cvtColor(legend_layer_, g, cv::COLOR_BGR2GRAY);
    cv::threshold(g, legend_mask_, 0, 255, cv::THRESH_BINARY);
  }

  void apply_legend(cv::Mat& img) const {
    cv::Mat roi = img(legend_rect_);
    roi.convertTo(roi, -1, 0.35, 0);  // dim the panel background
    legend_layer_.copyTo(roi, legend_mask_);
  }

  // ------------------------------------------------------------------ output
  static double now_steady_sec() {
    return std::chrono::duration<double>(SteadyClock::now().time_since_epoch())
        .count();
  }

  cv::Mat placeholder(const std::string& text) const {
    cv::Mat img = cv::Mat::zeros(out_size_, CV_8UC3);
    int base = 0;
    const cv::Size ts =
        cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.9, 2, &base);
    cv::putText(
        img, text,
        cv::Point((out_size_.width - ts.width) / 2, out_size_.height / 2),
        cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(255, 255, 255), 2,
        cv::LINE_AA);
    return img;
  }

  cv::Mat fresh(std::mutex& m, const TimedFrame& f) const {
    std::lock_guard<std::mutex> lk(m);
    if (f.img.empty()) {
      return cv::Mat();
    }
    const double age =
        std::chrono::duration<double>(SteadyClock::now() - f.arrived).count();
    if (stale_sec_ > 0.0 && age > stale_sec_) {
      return cv::Mat();
    }
    return f.img.clone();
  }

  bool selected_depth() const {
    const Mode m = mode_.load();
    if (m == Mode::ALTERNATE) {
      const double cs = alt_color_sec_.load(), ds = alt_depth_sec_.load();
      const double t = std::fmod(now_steady_sec() - alt_start_.load(), cs + ds);
      return t >= cs;
    }
    return m == Mode::DEPTH;
  }

  cv::Mat compose() {
    const bool show_depth = selected_depth();

    cv::Mat frame;
    if (show_depth) {
      frame = fresh(depth_mtx_, depth_viz_);
      if (frame.empty()) {
        frame = placeholder("NO DEPTH SIGNAL");
      }
    } else {
      frame = fresh(color_mtx_, color_);
      if (frame.empty()) {
        frame = placeholder("NO COLOR SIGNAL");
      }
    }
    if (overlay_mode_) {
      draw_label(frame, show_depth ? "DEPTH" : "COLOR", cv::Point(10, 26));
    }
    return frame;
  }

  // Fixed-rate mode: compose + send every tick. Event-driven mode: the timer (1
  // Hz) only retries opening the GStreamer pipeline; frames are sent from the
  // image callbacks.
  void on_timer() {
    if (event_driven_) {
      std::lock_guard<std::mutex> lk(writer_mtx_);
      if (gcs_enabled_ && !writer_.isOpened() &&
          now_steady_sec() - last_open_try_ > 5.0) {
        open_writer();
      }
      return;
    }
    emit_frame();
  }

  void emit_frame() {
    cv::Mat frame = compose();

    std::lock_guard<std::mutex> lk(writer_mtx_);
    if (gcs_enabled_) {
      if (!writer_.isOpened() && now_steady_sec() - last_open_try_ > 5.0) {
        open_writer();
      }
      if (writer_.isOpened()) {
        writer_.write(frame);
      }
    }
    if (pub_out_) {
      std_msgs::msg::Header h;
      h.stamp = now();
      h.frame_id = "visualizer";
      pub_out_->publish(*cv_bridge::CvImage(h, "bgr8", frame).toImageMsg());
    }
  }

  std::string build_pipeline() const {
    if (gcs_encoder_ == "custom") {
      return gcs_pipeline_;
    }
    const int fps = static_cast<int>(std::lround(fps_));
    std::string enc;
    if (gcs_encoder_ == "jetson") {
      enc =
          "appsrc is-live=true do-timestamp=true format=time ! queue ! "
          "videoconvert ! video/x-raw,format=I420 ! nvvidconv ! "
          "video/x-raw(memory:NVMM),format=NV12 ! nvv4l2h264enc bitrate=" +
          std::to_string(gcs_bitrate_ * 1000) +
          " insert-sps-pps=true idrinterval=" + std::to_string(fps) +
          " ! h264parse";
    } else if (gcs_encoder_ == "nvenc") {
      enc =
          "appsrc is-live=true do-timestamp=true format=time ! queue ! "
          "videoconvert ! nvh264enc bitrate=" +
          std::to_string(gcs_bitrate_) + " zerolatency=true ! h264parse";
    } else {
      enc =
          "appsrc is-live=true do-timestamp=true format=time ! queue ! "
          "videoconvert ! video/x-raw,format=I420 ! x264enc tune=zerolatency "
          "speed-preset=ultrafast bframes=0 bitrate=" +
          std::to_string(gcs_bitrate_) + " key-int-max=" + std::to_string(fps) +
          " ! video/x-h264,profile=baseline ! h264parse";
    }
    return enc +
           " config-interval=1 ! rtph264pay pt=96 config-interval=1 ! udpsink "
           "host=" +
           gcs_ip_ + " port=" + std::to_string(gcs_port_) +
           " sync=false async=false";
  }

  void open_writer() {
    last_open_try_ = now_steady_sec();
    const std::string pipe = build_pipeline();
    if (pipe.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "gcs.encoder=custom but gcs.pipeline is empty");
      return;
    }
    RCLCPP_INFO(get_logger(), "GStreamer: %s", pipe.c_str());
    if (!writer_.open(pipe, cv::CAP_GSTREAMER, 0, fps_, out_size_, true)) {
      RCLCPP_ERROR(get_logger(),
                   "could not open GStreamer pipeline (OpenCV built without "
                   "GStreamer, or missing plugins?). "
                   "Retrying every 5 s.");
      return;
    }
    RCLCPP_INFO(
        get_logger(),
        "streaming to %s:%d. Receive with:\n  gst-launch-1.0 udpsrc port=%d "
        "caps=\"application/x-rtp,media=video,encoding-name=H264,payload=96\" "
        "! rtph264depay ! "
        "avdec_h264 ! videoconvert ! autovideosink sync=false",
        gcs_ip_.c_str(), gcs_port_, gcs_port_);
  }

  // ------------------------------------------------------------------ members
  // params
  std::string topic_color_, topic_ir1_, topic_ir2_, topic_out_, topic_depth_;
  bool qos_reliable_{false};
  int qos_depth_{2};
  int cam_w_{848}, cam_h_{480};
  Intrinsics cam_l_, cam_r_;
  double baseline_{0.1};
  double proc_scale_{0.5};
  int num_disp_{208}, min_disp_{0}, block_size_{7}, p1_scale_{8}, p2_scale_{32};
  int uniq_{10}, speckle_win_{100}, speckle_rng_{2}, disp12_{1},
      prefilter_cap_{31}, median_k_{3};
  std::string sgbm_mode_;
  double slop_{0.02};
  bool pad_left_{true}, crop_strip_{false};
  double min_m_{0.1}, max_m_{3.0};
  int cmap_{cv::COLORMAP_JET};
  bool legend_enabled_{true};
  int legend_ticks_{6}, legend_w_{84};
  bool event_driven_{true};
  double fps_{15.0};
  cv::Size out_size_;
  double stale_sec_{1.0};
  bool overlay_mode_{true}, publish_ros_image_{false}, publish_depth_{false};
  bool gcs_enabled_{true};
  std::string gcs_ip_, gcs_encoder_, gcs_pipeline_;
  int gcs_port_{5600}, gcs_bitrate_{2000};

  // runtime-changeable
  std::atomic<Mode> mode_{Mode::COLOR};
  std::atomic<double> alt_color_sec_{5.0}, alt_depth_sec_{5.0}, alt_start_{0.0};
  std::atomic<bool> need_depth_{false};

  // stereo
  cv::Ptr<cv::StereoSGBM> matcher_;
  cv::Size maps_size_;
  cv::Mat map1_l_, map2_l_, map1_r_, map2_r_;
  std::mutex ir_mtx_;
  IrFrame ir_l_, ir_r_;

  // frames
  std::mutex color_mtx_, depth_mtx_;
  TimedFrame color_, depth_viz_;
  cv::Rect legend_rect_;
  cv::Mat legend_layer_, legend_mask_;

  // output
  std::mutex writer_mtx_;
  cv::VideoWriter writer_;
  double last_open_try_{0.0};

  // ROS
  rclcpp::CallbackGroup::SharedPtr cb_stereo_, cb_color_, cb_out_;
  rclcpp::Subscription<Image>::SharedPtr sub_color_, sub_ir1_, sub_ir2_;
  rclcpp::Publisher<Image>::SharedPtr pub_out_, pub_depth_;
  rclcpp::Service<SetStreamMode>::SharedPtr srv_mode_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Visualizer>();
    rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 4);
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("visualizer"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
