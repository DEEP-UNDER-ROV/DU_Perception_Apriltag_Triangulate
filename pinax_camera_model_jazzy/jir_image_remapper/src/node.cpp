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

/*
* Modular Image Remapper Node
*/

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <rmw/qos_profiles.h>

#include <jir_rectification_remap_lib/rectification_remapper.h>

class RemapperPipeline
{
public:
    RemapperPipeline(rclcpp::Node* node, 
                     const std::string& side, 
                     const std::string& map_path,
                     const std::string& in_topic,
                     const std::string& out_topic) 
        : node_(node), side_(side)
    {
        if (!rectifier_.loadMap(map_path)) {
            RCLCPP_ERROR(node_->get_logger(), "[%s] Failed to load map: %s", side_.c_str(), map_path.c_str());
            return;
        }
        camera_info_ = rectifier_.getCameraInfo();

        image_transport::ImageTransport it(node_->shared_from_this());
        
        std::string transport_hint = node_->get_parameter("image_transport").as_string();
        image_transport::TransportHints hints(node_, transport_hint);

        pub_ = it.advertiseCamera(out_topic, 1);

        RCLCPP_INFO(node_->get_logger(), "[%s] Subscribing to: %s (Transport: %s)", 
            side_.c_str(), in_topic.c_str(), transport_hint.c_str());

        sub_ = it.subscribe(
            in_topic,
            rmw_qos_profile_sensor_data, // Best Effort QoS
            &RemapperPipeline::callback,
            this,
            &hints
        );
    }

private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (rectifier_(cv_ptr->image, out_image_.image)) {
            out_image_.header = cv_ptr->header;
            out_image_.encoding = "bgr8";
            camera_info_.header = cv_ptr->header;
            pub_.publish(*out_image_.toImageMsg(), camera_info_);
        }
    }

    rclcpp::Node* node_;
    std::string side_;
    jir_rectification_remap_lib::RectificationRemapper rectifier_;
    sensor_msgs::msg::CameraInfo camera_info_;
    cv_bridge::CvImage out_image_;

    image_transport::CameraPublisher pub_;
    image_transport::Subscriber sub_;
};

class ModularRemapperNode : public rclcpp::Node 
{
public:
    ModularRemapperNode() : Node("jir_image_remapper")
    {
        this->declare_parameter("stereo", false);
        this->declare_parameter("image_transport", "raw");
        this->declare_parameter("left_map", "");
        this->declare_parameter("right_map", "");
        
        this->declare_parameter("left_input_topic", "in_left");
        this->declare_parameter("right_input_topic", "in_right");
        this->declare_parameter("left_output_topic", "out_left_rect/image_raw");
        this->declare_parameter("right_output_topic", "out_right_rect/image_raw");
    }

    void initialize() 
    {
        bool is_stereo = this->get_parameter("stereo").as_bool();
        
        auto left_map = this->get_parameter("left_map").as_string();
        
        auto left_in = this->get_parameter("left_input_topic").as_string();
        auto left_out = this->get_parameter("left_output_topic").as_string();

        left_pipe_ = std::make_shared<RemapperPipeline>(
            this, "Left", left_map, left_in, left_out
        );

        if (is_stereo) {
            auto right_map = this->get_parameter("right_map").as_string();
            auto right_in = this->get_parameter("right_input_topic").as_string();
            auto right_out = this->get_parameter("right_output_topic").as_string();

            right_pipe_ = std::make_shared<RemapperPipeline>(
                this, "Right", right_map, right_in, right_out
            );
        }
    }

private:
    std::shared_ptr<RemapperPipeline> left_pipe_;
    std::shared_ptr<RemapperPipeline> right_pipe_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ModularRemapperNode>();
    node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
