/*
 * Copyright (c) 2017 Jacobs University Robotics Group
 * All rights reserved.
 *
 *
 * Unless specified otherwise this code examples are released under
 * Creative Commons CC BY-NC-ND 4.0 license (free for non-commercial use).
 * Details may be found here: https://creativecommons.org/licenses/by-nc-nd/4.0/
 *
 *
 * If you are interested in using this code commercially,
 * please contact us.
 *
 * THIS SOFTWARE IS PROVIDED BY Jacobs Robotics ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Jacobs Robotics BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Contact: robotics@jacobs-university.de
 */

#include "defraction_map_finder/MapFinder.hpp"

#include <jir_refractive_image_geometry/ray.hpp>
#include <jir_refractive_image_geometry/ray_operations.hpp>
#include <jir_refractive_image_geometry/refraction_operations.hpp>

#include "defraction_map_finder/CameraFactory.h"

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace camodocal;

MapFinder::MapFinder(int W, int H, float d_0, float d0off, float d_1, float n_g,
                     float n_w, float nx, float ny, float nz, float proj_dist,
                     const sensor_msgs::msg::CameraInfo info,
                     CameraPtr realCam) {
  ImgWidth = W;
  ImgHeight = H;
  d0 = d_0;
  d0Virt_offset = d0off;
  d1 = d_1;
  ng = n_g;
  nw = n_w;
  nx_ = nx;
  ny_ = ny;
  nz_ = nz;
  proj_dist_ = proj_dist;

  Mat mx(ImgHeight, ImgWidth, DataType<float>::type);
  Mat my(ImgHeight, ImgWidth, DataType<float>::type);
  mapx = mx;
  mapy = my;

  VirtualCamera.fromCameraInfo(info);
  RealCamera.fromCameraInfo(info);

  jir_refractive_image_geometry_msgs::msg::PlanarRefractionInfo ref;
  ref.header = info.header;
  ref.normal[0] = nx_;
  ref.normal[1] = ny_;
  ref.normal[2] = nz_;
  ref.d_0 = d0;
  ref.d_1 = d1;
  ref.n_glass = ng;
  ref.n_water = nw;

  RefractionInfo = ref;
  RealCamera.fromPlanarRefractionInfo(ref);
  camera_info = info;
  myCamera = realCam;
}

void MapFinder::init_setup(int W, int H, float d_0, float d0off, float d_1,
                           float n_g, float n_w, float nx, float ny, float nz,
                           float proj_dist,
                           const sensor_msgs::msg::CameraInfo info,
                           CameraPtr realCam) {
  ImgWidth = W;
  ImgHeight = H;
  d0 = d_0;
  d0Virt_offset = d0off;
  d1 = d_1;
  ng = n_g;
  nw = n_w;

  nx_ = nx;
  ny_ = ny;
  nz_ = nz;
  proj_dist_ = proj_dist;

  Mat mx(ImgHeight, ImgWidth, DataType<float>::type);
  Mat my(ImgHeight, ImgWidth, DataType<float>::type);
  mapx = mx;
  mapy = my;

  VirtualCamera.fromCameraInfo(info);
  RealCamera.fromCameraInfo(info);

  jir_refractive_image_geometry_msgs::msg::PlanarRefractionInfo ref;
  ref.header = info.header;
  ref.normal[0] = nx_;
  ref.normal[1] = ny_;
  ref.normal[2] = nz_;
  ref.d_0 = d0;
  ref.d_1 = d1;
  ref.n_glass = ng;
  ref.n_water = nw;

  RefractionInfo = ref;

  RealCamera.fromPlanarRefractionInfo(ref);
  camera_info = info;
  myCamera = realCam;
}

void MapFinder::generate3Dpoints() {
  Vector3d plane_normal;
  plane_normal[0] = nx_;
  plane_normal[1] = ny_;
  plane_normal[2] = nz_;

  Eigen::Matrix3d R_rect;
  R_rect << camera_info.r[0], camera_info.r[1], camera_info.r[2],
      camera_info.r[3], camera_info.r[4], camera_info.r[5], camera_info.r[6],
      camera_info.r[7], camera_info.r[8];

  Eigen::Matrix3d R_rect_inv = R_rect.transpose();
  points3D.resize(ImgHeight * ImgWidth);

#pragma omp parallel for
  for (int i = 0; i < ImgHeight; i++) {
    for (int j = 0; j < ImgWidth; j++) {
      Ray<double> ray = VirtualCamera.projectPixelTo3dRay<double>(j, i);
      Eigen::Vector3d ray_vec(ray.direction[0], ray.direction[1],
                              ray.direction[2]);
      ray_vec = R_rect_inv * ray_vec;

      ray.direction[0] = ray_vec[0];
      ray.direction[1] = ray_vec[1];
      ray.direction[2] = ray_vec[2];

      ray.offset[2] += d0Virt_offset;

      Vector3d point;
      intersect(ray, plane_normal, static_cast<double>(proj_dist_), point);

      points3D[i * ImgWidth + j] = point;
    }
  }
}

void MapFinder::projectPoints() {
  Vector3d rotatedNormal(nx_, ny_, nz_);

  rotatedNormal.normalize();
  RefractionInfo.normal[0] = rotatedNormal[0];
  RefractionInfo.normal[1] = rotatedNormal[1];
  RefractionInfo.normal[2] = rotatedNormal[2];

#pragma omp parallel for
  for (int i = 0; i < ImgHeight; i++) {
    for (int j = 0; j < ImgWidth; j++) {
      Point2d temp;
      bool notRef = false;

      Vector3d point3d = points3D[ImgWidth * i + j];

      if (point3d[0] == 0 && point3d[1] == 0) {
        notRef = true;
      }
      temp = projectPoint(point3d, notRef);

      mapx.at<float>(i, j) = temp.x;
      mapy.at<float>(i, j) = temp.y;
    }
  }
}

cv::Mat MapFinder::getMask() const {
  Mat mask(myCamera->imageHeight(), myCamera->imageWidth(), CV_8UC1,
           Scalar::all(255));

  Mat mask_out;

  cv::remap(mask, mask_out, mapx, mapy, cv::INTER_LINEAR, BORDER_CONSTANT,
            Scalar::all(0));

  Mat mask_thresh;
  threshold(mask_out, mask_thresh, 250, 255, THRESH_BINARY);

  return mask_thresh;
}

void MapFinder::saveMaps(const std::string& filename) {
  cv::Mat mask = getMask();

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
    mask = clean_mask;

    mapx.setTo(-1.0f, mask == 0);
    mapy.setTo(-1.0f, mask == 0);
  }

  FileStorage fs(filename, FileStorage::WRITE);

  Mat K(3, 3, DataType<float>::type);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      K.at<float>(i, j) = camera_info.k[i * 3 + j];
    }
  }

  fs << "camera_info" << K;

  fs << "input_rows" << myCamera->imageHeight();
  fs << "input_cols" << myCamera->imageWidth();
  fs << "mapx" << mapx;
  fs << "mapy" << mapy;
  fs << "mask" << mask;
}

Point2d MapFinder::projectPoint(Vector3d point3D, bool notRefracted) {
  Vector2d pointDistorted;
  Vector3d xyz;

  if (notRefracted) {
    return Point2d(-1.0, -1.0);
  }

  bool ok = point_on_inside_of_refractive_plane_analytic(point3D,
                                                         RefractionInfo, xyz);
  if (!ok) {
    return Point2d(-1.0, -1.0);
  }

  myCamera->spaceToPlane(xyz, pointDistorted);
  Point2d tmp(pointDistorted[0], pointDistorted[1]);
  return tmp;
}
