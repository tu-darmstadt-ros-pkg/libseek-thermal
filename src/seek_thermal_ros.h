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


#include "seek.h"

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "image_transport/image_transport.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <boost/shared_ptr.hpp>

class SeekThermalRos
{
public:
  SeekThermalRos(const rclcpp::NodeOptions& node_options = rclcpp::NodeOptions());
  void frameGrabTimerCallback();
  void tryOpenDeviceTillSuccess();

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const;

protected:
  image_transport::Publisher image_pub_;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_pub_;
  rclcpp::TimerBase::SharedPtr frame_grab_timer_;

  cv_bridge::CvImage cv_image_;  

  sensor_msgs::msg::CameraInfo camera_info_;  

  std::shared_ptr<LibSeek::SeekCam> seek_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;

  // Diagnostics
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  std::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> img_pub_freq_;
  double diagnostics_freq_min_;
  double diagnostics_freq_max_;

  // Params
  int rotate90_;
  int device_index_;
  std::string frame_id_;
  std::string camera_info_url_;
  std::string cam_name_;
  std::string flat_field_calibration_path_;
  bool is_seek_pro_;

private:
  std::shared_ptr<rclcpp::Node> node_;
};
