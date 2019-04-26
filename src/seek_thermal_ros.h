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


#ifndef SEEK_THERMAL_ROS_H_____
#define SEEK_THERMAL_ROS_H_____

#include "seek.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>

#include <camera_info_manager/camera_info_manager.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>



class SeekThermalRos
{
public:

  SeekThermalRos(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  void frameGrabTimerCallback(const ros::TimerEvent& event);

protected:

  image_transport::Publisher image_pub_;
  ros::Publisher cam_info_pub_;

  cv_bridge::CvImage cv_image_;
  sensor_msgs::CameraInfo camera_info_;

  boost::shared_ptr<LibSeek::SeekThermal> seek_;
  boost::shared_ptr <camera_info_manager::CameraInfoManager> camera_info_manager_;

  ros::Timer frame_grab_timer_;

  // Diagnostics
  boost::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  boost::shared_ptr<diagnostic_updater::HeaderlessTopicDiagnostic> img_pub_freq_;
  double diagnostics_freq_min_;
  double diagnostics_freq_max_;
  
  // Params
  int rotate90_;
  std::string frame_id_;
  std::string camera_info_url_;
  std::string cam_name_;


};

#endif
