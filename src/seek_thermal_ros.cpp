//=================================================================================================
// Copyright (c) 2019, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================


#include "seek_thermal_ros.h"

SeekThermalRos::SeekThermalRos(const rclcpp::NodeOptions& node_options) : node_(std::make_shared<rclcpp::Node>("seek_thermal_ros", node_options))
{        
    RCLCPP_INFO(node_->get_logger(), "Seek Thermal ROS node started");
    node_->declare_parameter("rotate90", 0);
    node_->declare_parameter("device_index", 0);
    node_->declare_parameter("frame_id", std::string("seek_thermal_frame"));
    node_->declare_parameter("camera_info_url", std::string());
    node_->declare_parameter("camera_name", std::string("seek_thermal"));
    node_->declare_parameter("flat_field_calibration_path", std::string());
    node_->declare_parameter("is_seek_pro", false);
    node_->declare_parameter("diagnostics_freq_min", 5.0);
    node_->declare_parameter("diagnostics_freq_max", 11.0);

    node_->get_parameter("rotate90", rotate90_);
    node_->get_parameter("device_index", device_index_);
    node_->get_parameter("frame_id", frame_id_);
    node_->get_parameter("camera_info_url", camera_info_url_);
    node_->get_parameter("camera_name", cam_name_);
    node_->get_parameter("flat_field_calibration_path", flat_field_calibration_path_);
    node_->get_parameter("is_seek_pro", is_seek_pro_);
    node_->get_parameter("diagnostics_freq_min", diagnostics_freq_min_);
    node_->get_parameter("diagnostics_freq_max", diagnostics_freq_max_);
  
    camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(node_.get(), cam_name_, camera_info_url_);
    
    camera_info_ = camera_info_manager_->getCameraInfo();
    camera_info_.header.frame_id = frame_id_;

    image_transport::ImageTransport it(node_->shared_from_this());

    image_pub_ = it.advertise("camera/image", 1);
    cam_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>("camera/camera_info", 5);

    if (is_seek_pro_)
      seek_ = std::make_shared<LibSeek::SeekThermalPro>(flat_field_calibration_path_, device_index_);
    else
      seek_ = std::make_shared<LibSeek::SeekThermal>(flat_field_calibration_path_, device_index_);

    this->tryOpenDeviceTillSuccess();
    frame_grab_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100), std::bind(&SeekThermalRos::frameGrabTimerCallback, this));

    cv_image_.encoding = sensor_msgs::image_encodings::MONO16;
    cv_image_.header.frame_id = frame_id_;

    diagnostic_updater_.reset(new diagnostic_updater::Updater(node_.get())); // default period: 1s
    diagnostic_updater_->setHardwareID(cam_name_);

    img_pub_freq_.reset(new diagnostic_updater::HeaderlessTopicDiagnostic("Image Pub Frequency",
           *diagnostic_updater_,
           diagnostic_updater::FrequencyStatusParam(&diagnostics_freq_min_, &diagnostics_freq_max_, 0.0)));
}

// Required for non-node derived components
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr SeekThermalRos::get_node_base_interface() const
{
    return this->node_->get_node_base_interface();
}

void SeekThermalRos::tryOpenDeviceTillSuccess()
{
    while (!seek_->open()){
      RCLCPP_WARN(node_->get_logger(), "Failed to open Seek Thermal device, retrying in 5s!");
      rclcpp::sleep_for(std::chrono::seconds(5));
    }
    RCLCPP_INFO(node_->get_logger(), "Successfully opened Seek Thermal device!");
}

void SeekThermalRos::frameGrabTimerCallback()
{
    // Read directly into cv::Mat in cv_image
    if (!seek_->read(cv_image_.image)){
      seek_.reset();

    if (is_seek_pro_)
      seek_ = std::make_shared<LibSeek::SeekThermalPro>(flat_field_calibration_path_, device_index_);
    else
      seek_ = std::make_shared<LibSeek::SeekThermal>(flat_field_calibration_path_, device_index_);

    this->tryOpenDeviceTillSuccess();
  }
  
  if (rotate90_ == 1)
    cv::rotate(cv_image_.image, cv_image_.image,0);
  else if (rotate90_ == 2)
      cv::rotate(cv_image_.image, cv_image_.image,1);
  else if (rotate90_ == 3)
      cv::rotate(cv_image_.image, cv_image_.image,2);
  
  rclcpp::Time retrieve_time = node_->now();  

  cv_image_.header.stamp = retrieve_time;

  // Creates a shared ptr copy of image and publishes
  image_pub_.publish(cv_image_.toImageMsg());

  img_pub_freq_->tick();
  // diagnostic_updater_->update();
  
  if (cam_info_pub_->get_subscription_count() > 0) {
    camera_info_.header.stamp = retrieve_time;
    cam_info_pub_->publish(camera_info_);
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(SeekThermalRos)