/*
 * Standalone mask-dump tool.
 *
 * --- Modified: removed hardcoded fallback defaults (matches main.cpp fix --
 * --- every housing/intrinsic field must be present in config.yaml or this
 * --- aborts with a named error). Also applies the same largest-blob +
 * --- erosion margin cleanup that MapFinder::saveMaps() uses, so this PNG
 * --- actually reflects what the LUT will produce -- previously it dumped
 * --- the raw un-eroded mask, which is why it looked clean while the real
 * --- rectified output still leaked contaminated pixels at the boundary.
 */
#include <iostream>
#include <jir_refractive_image_geometry/refracted_pinhole_camera_model.hpp>
#include <opencv2/ml.hpp>
#include <opencv2/opencv.hpp>

#include "defraction_map_finder/CameraFactory.h"
#include "defraction_map_finder/MapFinder.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

using namespace std;

static bool requireFloat(const cv::FileNode& node, const char* key,
                         float& dst) {
  if (node[key].empty()) {
    cerr << "ERROR: required field '" << key << "' is missing from config."
         << endl;
    return false;
  }
  dst = (float)node[key];
  return true;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    cout << "Usage: " << argv[0]
         << " <config.yaml> <output_mask.png> [margin_px]\n";
    return 1;
  }
  string intr = argv[1];
  string outMaskPath = argv[2];
  int margin = (argc >= 4) ? atoi(argv[3]) : 3;

  camodocal::CameraPtr distCam =
      camodocal::CameraFactory::instance()->generateCameraFromYamlFile(intr);
  if (!distCam) {
    cerr << "ERROR: Could not load camera calibration from " << intr << endl;
    return 1;
  }

  cv::FileStorage fs(intr, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    cerr << "ERROR: Could not open config file " << intr << endl;
    return 1;
  }

  if (fs["image_width"].empty() || fs["image_height"].empty()) {
    cerr << "ERROR: config must specify image_width and image_height." << endl;
    return 1;
  }
  int W = (int)fs["image_width"];
  int H = (int)fs["image_height"];

  float fx, fy, cx, cy;
  cv::FileNode proj = fs["projection_parameters"];
  if (proj.empty()) {
    cerr << "ERROR: config must specify projection_parameters (fx, fy, cx, cy)."
         << endl;
    return 1;
  }
  bool ok = true;
  ok &= requireFloat(proj, "fx", fx);
  ok &= requireFloat(proj, "fy", fy);
  ok &= requireFloat(proj, "cx", cx);
  ok &= requireFloat(proj, "cy", cy);
  if (!ok) return 1;

  cv::FileNode housing = fs["housing_parameters"];
  if (housing.empty()) {
    cerr << "ERROR: config must specify a housing_parameters block "
            "(d0, d0_offset, d1, n_glass, n_water, nx, ny, nz, proj_dist). "
            "Refusing to fall back to generic defaults."
         << endl;
    return 1;
  }

  float d_0, d0off, d_1, n_g, n_w, nx, ny, nz, proj_dist;
  ok = true;
  ok &= requireFloat(housing, "d0", d_0);
  ok &= requireFloat(housing, "d0_offset", d0off);
  ok &= requireFloat(housing, "d1", d_1);
  ok &= requireFloat(housing, "n_glass", n_g);
  ok &= requireFloat(housing, "n_water", n_w);
  ok &= requireFloat(housing, "nx", nx);
  ok &= requireFloat(housing, "ny", ny);
  ok &= requireFloat(housing, "nz", nz);
  ok &= requireFloat(housing, "proj_dist", proj_dist);
  if (!ok) {
    cerr
        << "ERROR: one or more housing_parameters fields are missing. Aborting."
        << endl;
    return 1;
  }

  cout << "=== Mask Dump ===" << endl;
  cout << "Image size:  " << W << "x" << H << endl;
  cout << "Intrinsics:  fx=" << fx << " fy=" << fy << " cx=" << cx
       << " cy=" << cy << endl;
  cout << "Housing:     d0=" << d_0 << " d0_offset=" << d0off << " d1=" << d_1
       << " n_glass=" << n_g << " n_water=" << n_w << endl;
  cout << "Normal:      (" << nx << ", " << ny << ", " << nz << ")" << endl;
  cout << "proj_dist:   " << proj_dist << endl;
  cout << "mask margin: " << margin << " px" << endl;
  cout << "=================" << endl;

  sensor_msgs::msg::CameraInfo info;
  info.height = H;
  info.width = W;
  info.k[0] = fx;
  info.k[1] = 0;
  info.k[2] = cx;
  info.k[3] = 0;
  info.k[4] = fy;
  info.k[5] = cy;
  info.k[6] = 0;
  info.k[7] = 0;
  info.k[8] = 1;

  cv::FileNode rect_rot = fs["rectification_matrix"];
  if (!rect_rot.empty()) {
    for (int i = 0; i < 9; i++) info.r[i] = (double)rect_rot["data"][i];
  } else {
    info.r[0] = 1.0;
    info.r[4] = 1.0;
    info.r[8] = 1.0;
  }
  fs.release();

  info.p[0] = fx;
  info.p[2] = cx;
  info.p[5] = fy;
  info.p[6] = cy;
  info.p[10] = 1.0;
  info.roi.height = H;
  info.roi.width = W;
  info.roi.do_rectify = false;

  MapFinder find(W, H, d_0, d0off, d_1, n_g, n_w, nx, ny, nz, proj_dist, info,
                 distCam);
  find.generate3Dpoints();
  find.projectPoints();

  cv::Mat mask = find.getMask();

  // Same cleanup as MapFinder::saveMaps(): keep only the largest connected
  // blob, then erode it by `margin` px so this PNG actually shows what the
  // LUT will reject -- not the raw, un-eroded boundary.
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (!contours.empty()) {
    int largest_idx = 0;
    double largest_area = 0;
    for (size_t i = 0; i < contours.size(); i++) {
      double area = cv::contourArea(contours[i]);
      if (area > largest_area) {
        largest_area = area;
        largest_idx = i;
      }
    }
    cv::Mat clean_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
    cv::drawContours(clean_mask, contours, largest_idx, cv::Scalar(255),
                     cv::FILLED);

    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * margin + 1, 2 * margin + 1));
    cv::erode(clean_mask, clean_mask, kernel);

    mask = clean_mask;
  }

  cout << "Mask size: " << mask.cols << " x " << mask.rows << endl;
  cv::imwrite(outMaskPath, mask);
  cout << "Saved mask to " << outMaskPath << endl;

  return 0;
}
