// Copyright (c) 2015, JSK Lab
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the JSK Lab nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "tf_trajectory_display.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>

#include <sstream>
#include <string>

namespace tf_trajectory_rviz_plugin
{
TFTrajectoryDisplay::TFTrajectoryDisplay() : Display(), clock_(RCL_SYSTEM_TIME)
{
  frame_property_ = new rviz_common::properties::TfFrameProperty(
    "frame", "", "TF frame to visualize trajectory", this, nullptr, false, SLOT(updateFrame()));
  duration_property_ = new rviz_common::properties::FloatProperty(
    "duration", 10.0, "Duration (seconds) of trajectory to keep", this, SLOT(updateDuration()));
  line_width_property_ = new rviz_common::properties::FloatProperty(
    "line_width", 0.01, "Line width", this, SLOT(updateLineWidth()));
  color_property_ = new rviz_common::properties::ColorProperty(
    "color", QColor(25, 255, 240), "Color of trajectory line", this, SLOT(updateColor()));
  duration_property_->setMin(0.0);
  line_width_property_->setMin(0.0);
}

TFTrajectoryDisplay::~TFTrajectoryDisplay()
{
  delete line_;
  delete line_width_property_;
  delete frame_property_;
  delete duration_property_;
  delete color_property_;
}

void TFTrajectoryDisplay::onInitialize()
{
  frame_property_->setFrameManager(context_->getFrameManager());
  line_ = new rviz_rendering::BillboardLine(context_->getSceneManager(), scene_node_);
  updateFrame();
  updateDuration();
  updateColor();
  updateLineWidth();
}

void TFTrajectoryDisplay::updateFrame()
{
  frame_ = frame_property_->getFrame().toStdString();
  trajectory_.clear();
}

void TFTrajectoryDisplay::updateDuration() { duration_ = duration_property_->getFloat(); }

void TFTrajectoryDisplay::updateColor() { color_ = color_property_->getColor(); }

void TFTrajectoryDisplay::onEnable()
{
  line_->clear();
  trajectory_.clear();
}

void TFTrajectoryDisplay::updateLineWidth() { line_width_ = line_width_property_->getFloat(); }

void TFTrajectoryDisplay::onDisable()
{
  line_->clear();
  trajectory_.clear();
}

void TFTrajectoryDisplay::update(float wall_dt, float ros_dt)
{
  if (frame_.empty()) {
    return;
  }
  std::string fixed_frame_id = context_->getFrameManager()->getFixedFrame();
  if (fixed_frame_ != fixed_frame_id) {
    fixed_frame_ = fixed_frame_id;
    line_->clear();
    trajectory_.clear();
    return;
  }
  fixed_frame_ = fixed_frame_id;
  rclcpp::Time now = context_->getFrameManager()->getTime();
  std_msgs::msg::Header header;
  header.stamp = rclcpp::Time(0, 0, clock_.get_clock_type());
  header.frame_id = frame_;
  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(header, position, orientation)) {
    std::ostringstream oss;
    oss << "Failed transforming from frame '" << header.frame_id
        << "' to frame '" << fixed_frame_id << "'";
    setStatus(
      rviz_common::properties::StatusProperty::Error, "transformation",
      QString::fromStdString(oss.str()));
    return;
  }
  setStatus(rviz_common::properties::StatusProperty::Ok, "transformation", "Ok");
  geometry_msgs::msg::PointStamped new_point;
  new_point.header.stamp = now;
  new_point.point.x = position[0];
  new_point.point.y = position[1];
  new_point.point.z = position[2];
  trajectory_.push_back(new_point);

  for (auto it = trajectory_.begin(); it != trajectory_.end();) {
    rclcpp::Duration duration = now - it->header.stamp;
    if (duration.seconds() > duration_) {
      it = trajectory_.erase(it);
    } else {
      break;
    }
  }
  line_->clear();
  line_->setNumLines(1);
  line_->setMaxPointsPerLine(trajectory_.size());
  line_->setLineWidth(line_width_);
  line_->setColor(
    color_.redF(), color_.greenF(), color_.blueF(), 1.0f);
  for (size_t i = 0; i < trajectory_.size(); i++) {
    Ogre::Vector3 p;
    p[0] = trajectory_[i].point.x;
    p[1] = trajectory_[i].point.y;
    p[2] = trajectory_[i].point.z;
    line_->addPoint(p);
  }
}
}  // namespace tf_trajectory_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(tf_trajectory_rviz_plugin::TFTrajectoryDisplay, rviz_common::Display)
