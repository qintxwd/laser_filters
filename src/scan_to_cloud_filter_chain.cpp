/*
 * Copyright (c) 2008 Radu Bogdan Rusu <rusu@cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: scan_shadows_filter.cpp,v 1.0 2008/12/04 12:00:00 rusu Exp $
 *
 */

/*
\author Radu Bogdan Rusu <rusu@cs.tum.edu>


 */

#include "scan_to_cloud_filter_chain.h"

namespace laser_filters
{

ScanToCloudFilterChain::ScanToCloudFilterChain(const rclcpp::NodeOptions &options)
  : rclcpp::Node("scan_to_cloud_filter_chain_node", options)
  , laser_max_range_(DBL_MAX)
{
  declare_parameter("high_fidelity", false);
  declare_parameter("notifier_tolerance", 0.03);
  declare_parameter("target_frame", std::string("base_link"));
  declare_parameter("incident_angle_correction", true);

  get_parameter("high_fidelity", high_fidelity_);
  get_parameter("notifier_tolerance", tf_tolerance_);
  get_parameter("target_frame", target_frame_);
  get_parameter("incident_angle_correction", incident_angle_correction_);

  get_parameter_or("filter_window", window_, 2);
  get_parameter_or("laser_max_range", laser_max_range_, DBL_MAX);
  get_parameter_or("scan_topic", scan_topic_, std::string("tilt_scan"));
  get_parameter_or("cloud_topic", cloud_topic_, std::string("tilt_laser_cloud_filtered"));

  buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

  sub_ =
      std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(shared_from_this(), "scan", rmw_qos_profile_sensor_data);

  filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(*sub_, *buffer_, "", 50, shared_from_this());

  cloud_filter_chain_ = std::make_shared<filters::FilterChain<sensor_msgs::msg::PointCloud2>>("sensor_msgs::msg::PointCloud2");

  scan_filter_chain_ = std::make_shared<filters::FilterChain<sensor_msgs::msg::LaserScan>>("sensor_msgs::msg::LaserScan");

  filter_->setTargetFrame(target_frame_);
  filter_->registerCallback(std::bind(&ScanToCloudFilterChain::scanCallback, this, std::placeholders::_1));
  filter_->setTolerance(std::chrono::duration<double>(tf_tolerance_));

  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface());
  buffer_->setCreateTimerInterface(timer_interface);

  sub_->subscribe(shared_from_this(), "scan", rmw_qos_profile_sensor_data);

  filter_->connectInput(*sub_);

  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_filtered", 10);

  cloud_filter_chain_->configure("cloud_filter_chain", get_node_logging_interface(), get_node_parameters_interface());

  scan_filter_chain_->configure("scan_filter_chain", get_node_logging_interface(), get_node_parameters_interface());
}

////////////////////////////////////////////////////////////////////////////////
void ScanToCloudFilterChain::scanCallback(const std::shared_ptr<const sensor_msgs::msg::LaserScan> &scan_msg)
{
  //    sensor_msgs::msg::LaserScan scan_msg = *scan_in;

  sensor_msgs::msg::LaserScan filtered_scan;
  scan_filter_chain_->update(*scan_msg, filtered_scan);

  // Project laser into point cloud
  sensor_msgs::msg::PointCloud2 scan_cloud;

  //\TODO CLEAN UP HACK
  // This is a trial at correcting for incident angles.  It makes many assumptions that do not generalise
  if (incident_angle_correction_)
  {
    for (unsigned int i = 0; i < filtered_scan.ranges.size(); i++)
    {
      double angle = filtered_scan.angle_min + i * filtered_scan.angle_increment;
      filtered_scan.ranges[i] = filtered_scan.ranges[i] + 0.03 * exp(-fabs(sin(angle)));
    }
  }

  // Transform into a PointCloud message
  int mask = laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance | laser_geometry::channel_option::Index |
             laser_geometry::channel_option::Timestamp;

  if (high_fidelity_)
  {
    try
    {
      projector_.transformLaserScanToPointCloud(target_frame_, filtered_scan, scan_cloud, *buffer_, mask);
    }
    catch (tf2::TransformException &ex)
    {
      RCLCPP_WARN(get_logger(), "High fidelity enabled, but TF returned a transform exception to frame %s: %s", target_frame_.c_str(),
                  ex.what());
      return;
      // projector_.projectLaser (filtered_scan, scan_cloud, laser_max_range_, preservative_, mask);
    }
  }
  else
  {
    projector_.transformLaserScanToPointCloud(target_frame_, filtered_scan, scan_cloud, *buffer_, laser_max_range_, mask);
  }

  sensor_msgs::msg::PointCloud2 filtered_cloud;
  cloud_filter_chain_->update(scan_cloud, filtered_cloud);

  cloud_pub_->publish(filtered_cloud);
}

}  // namespace laser_filters

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(laser_filters::ScanToCloudFilterChain)