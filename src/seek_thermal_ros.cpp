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

SeekThermalRos::SeekThermalRos(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{


  pnh.param("rotate90", rotate90_, 0);
  pnh.param("device_index", device_index_, 0);
  pnh.param("frame_id", frame_id_, std::string("seek_thermal_frame"));
  pnh.param("camera_info_url", camera_info_url_, std::string());
  pnh.param("camera_name", cam_name_, std::string("seek_thermal"));
  pnh.param("flat_field_calibration_path", flat_field_calibration_path_, std::string());

  camera_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(nh, cam_name_, camera_info_url_);
  
  camera_info_ = camera_info_manager_->getCameraInfo();
  camera_info_.header.frame_id = frame_id_;
  
  image_transport::ImageTransport it(nh);

  image_pub_ = it.advertise("camera/image", 1);  
  cam_info_pub_ = nh.advertise<sensor_msgs::CameraInfo>("camera/camera_info",5);

  seek_ = boost::make_shared<LibSeek::SeekThermal>(flat_field_calibration_path_, device_index_);

  this->tryOpenDeviceTillSuccess();

  frame_grab_timer_ = nh.createTimer(ros::Duration(0.1), &SeekThermalRos::frameGrabTimerCallback, this);

  // Could also use camera pub, but maybe we want to not publish CameraInfo
  // msgs all the time if we don't ever use them anyways
  //cam_still_pub_ = it.advertiseCamera("camera/still/image", 1);

  cv_image_.encoding = sensor_msgs::image_encodings::MONO16;
  cv_image_.header.frame_id = frame_id_;

  //Diagnostics
  pnh.param<double>("diagnostics_freq_min", diagnostics_freq_min_, 6.0);
  pnh.param<double>("diagnostics_freq_max", diagnostics_freq_max_, 10.0);

  diagnostic_updater_.reset(new diagnostic_updater::Updater);
  //@TODO: Proper camera name
  diagnostic_updater_->setHardwareID(pnh.getNamespace());

  img_pub_freq_.reset(new diagnostic_updater::HeaderlessTopicDiagnostic("Image Pub Frequency",
           *diagnostic_updater_,
           diagnostic_updater::FrequencyStatusParam(&diagnostics_freq_min_, &diagnostics_freq_max_, 0.0)));
}

void SeekThermalRos::tryOpenDeviceTillSuccess()
{
  while (!seek_->open()){
    ROS_WARN("Failed to open Seek Thermal device, retrying in 5s!");
    ros::Duration(5.0).sleep();
  }
  ROS_INFO("Successfully opened Seek Thermal device!");
}

void SeekThermalRos::frameGrabTimerCallback(const ros::TimerEvent& event)
{
  // Read directly into cv::Mat in cv_image
  if (!seek_->read(cv_image_.image)){
    seek_.reset();
    seek_ = boost::make_shared<LibSeek::SeekThermal>(device_index_);
    this->tryOpenDeviceTillSuccess();
  }
  
  if (rotate90_ == 1)
    cv::rotate(cv_image_.image, cv_image_.image,0);
  else if (rotate90_ == 2)
      cv::rotate(cv_image_.image, cv_image_.image,1);
  else if (rotate90_ == 3)
      cv::rotate(cv_image_.image, cv_image_.image,2);
  
  ros::Time retrieve_time = ros::Time::now();

  cv_image_.header.stamp = retrieve_time;

  // Creates a shared ptr copy of image and publishes
  image_pub_.publish(cv_image_.toImageMsg());

  img_pub_freq_->tick();
  diagnostic_updater_->update();
  
  if (cam_info_pub_.getNumSubscribers() > 0){
    camera_info_.header.stamp = retrieve_time;
    cam_info_pub_.publish(camera_info_);
  }
    
}


