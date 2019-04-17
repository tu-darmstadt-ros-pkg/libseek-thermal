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
  image_transport::ImageTransport it(nh);

  image_pub_ = it.advertise("camera/image", 1);

  seek_ = boost::make_shared<LibSeek::SeekThermal>();

  if (!seek_->open()){
    ROS_ERROR("Failed to open device!");
    //return;
  }

  frame_grab_timer_ = nh.createTimer(ros::Duration(0.1), &SeekThermalRos::frameGrabTimerCallback, this);

  // Could also use camera pub, but maybe we want to not publish CameraInfo
  // msgs all the time if we don't ever use them anyways
  //cam_still_pub_ = it.advertiseCamera("camera/still/image", 1);

  cv_image_.encoding = sensor_msgs::image_encodings::MONO16;
  cv_image_.header.frame_id = "todo_fill_me_via_param";
}

void SeekThermalRos::frameGrabTimerCallback(const ros::TimerEvent& event)
{
  // Read directly into cv::Mat in cv_image
  seek_->read(cv_image_.image);

  ros::Time retrieve_time = ros::Time::now();

  cv_image_.header.stamp = retrieve_time;

  // Creates a shared ptr copy of image and publishes
  image_pub_.publish(cv_image_.toImageMsg());
}


