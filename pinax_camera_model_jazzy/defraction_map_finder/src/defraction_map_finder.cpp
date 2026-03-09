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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <fstream>
#include <string>
#include <jir_refractive_image_geometry/refracted_pinhole_camera_model.hpp>
#include "defraction_map_finder/MapFinder.hpp"
#include "defraction_map_finder/CameraFactory.h"
#include <Eigen/Geometry>

using namespace cv;
using namespace std;
using namespace Eigen;
using namespace camodocal;

void printUsage(const char* prog) {
    cout << "Usage: " << prog << " <config.yaml> <output_map.yaml>\n"
         << "\nArguments:\n"
         << "  <config.yaml>      Camera calibration with housing parameters\n"
         << "  <output_map.yaml>  Output correction map file\n"
         << "\nConfig file should include housing_parameters:\n"
         << "  housing_parameters:\n"
         << "     d0: 0.005        # Air gap (meters)\n"
         << "     d1: 0.005        # Glass thickness (meters)\n"
         << "     n_glass: 1.49\n"
         << "     n_water: 1.33\n";
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        printUsage(argv[0]);
        return 1;
    }

    // Input/output files from command line
    string intr = argv[1];
    string mapsResults = argv[2];

    // Load camera calibration
    camodocal::CameraPtr distCam = CameraFactory::instance()->generateCameraFromYamlFile(intr);
    if (!distCam) {
        cerr << "ERROR: Could not load camera calibration from " << intr << endl;
        return 1;
    }

    // Read all parameters from config file
    cv::FileStorage fs(intr, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "ERROR: Could not open config file " << intr << endl;
        return 1;
    }

    // Get image dimensions
    int W = (int)fs["image_width"];
    int H = (int)fs["image_height"];

    // Get focal length from projection_parameters
    float fx, fy, cx, cy;  // default
    cv::FileNode proj = fs["projection_parameters"];
    if (!proj.empty()) {
        fx = (float)proj["fx"];
        fy = (float)proj["fy"];
        cx = (float)proj["cx"];
        cy = (float)proj["cy"];
    }

    // Default housing parameters
    float d_0 = 0.005;
    float d0off = 0.0008;
    float d_1 = 0.005;
    float n_g = 1.49;
    float n_w = 1.33;

    // Read housing parameters
    cv::FileNode housing = fs["housing_parameters"];
    if (!housing.empty()) {
        if (!housing["d0"].empty()) d_0 = (float)housing["d0"];
        if (!housing["d1"].empty()) d_1 = (float)housing["d1"];
        if (!housing["n_glass"].empty()) n_g = (float)housing["n_glass"];
        if (!housing["n_water"].empty()) n_w = (float)housing["n_water"];
    } else {
        cout << "WARNING: No housing_parameters in config, using defaults" << endl;
    }


    // Print configuration
    cout << "=== Defraction Map Finder ===" << endl;
    cout << "Input:         " << intr << endl;
    cout << "Output:        " << mapsResults << endl;
    cout << "Image size:    " << W << "x" << H << endl;
    cout << "Virtual camera intrinsics:" << endl;
    cout << "  fx: " << fx << endl;
    cout << "  fy: " << fy << endl;
    cout << "  cx: " << cx << endl;
    cout << "  cy: " << cy << endl;
    cout << "Housing:" << endl;
    cout << "  d0: " << d_0 << " m" << endl;
    cout << "  d1: " << d_1 << " m" << endl;
    cout << "  n_glass: " << n_g << endl;
    cout << "  n_water: " << n_w << endl;
    cout << "==============================" << endl;

    // Create camera info
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
    if(!rect_rot.empty()) {
        info.r[0] = (double)rect_rot["data"][0]; info.r[1] = (double)rect_rot["data"][1]; info.r[2] = (double)rect_rot["data"][2];
        info.r[3] = (double)rect_rot["data"][3]; info.r[4] = (double)rect_rot["data"][4]; info.r[5] = (double)rect_rot["data"][5];
        info.r[6] = (double)rect_rot["data"][6]; info.r[7] = (double)rect_rot["data"][7]; info.r[8] = (double)rect_rot["data"][8];
        cout << "Loaded Stereo Rectification Matrix." << endl;
    } else {
        cout << "WARNING: No rectification_matrix found. Defaulting to Identity (Mono only!)." << endl;
        info.r[0] = 1.0; info.r[4] = 1.0; info.r[8] = 1.0;
    }

    fs.release();

    info.p[0] = fx;
    info.p[2] = cx;
    info.p[5] = fy;
    info.p[6] = cy;
    info.p[10] = 1.0;

    info.binning_x = 0;
    info.binning_y = 0;
    info.roi.height = H;
    info.roi.width = W;
    info.roi.do_rectify = false;

    // Generate maps
    MapFinder find(W, H, d_0, d0off, d_1, n_g, n_w, info, distCam);
    cout << "Object created" << endl;

    find.generate3Dpoints();
    cout << "3D points generated" << endl;

    find.projectPoints();
    cout << "Points projected" << endl;

    find.saveMaps(mapsResults);
    cout << "Results saved to " << mapsResults << endl;

    return 0;
}
