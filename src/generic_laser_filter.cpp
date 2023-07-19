/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 */

#include "generic_laser_filter.h"

namespace laser_filters
{

void GenericLaserScanFilter::foo(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
}

GenericLaserScanFilter::GenericLaserScanFilter(const rclcpp::NodeOptions &options)
  : rclcpp::Node("generic_laser_scan_filter_node", options)
{
  buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());

  tf_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);

  scan_sub_ =
      std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(shared_from_this(), "scan", rmw_qos_profile_sensor_data);

  tf_filter_ =
      std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(*scan_sub_, *buffer_, "base_link", 50, shared_from_this());

  filter_chain_ = std::make_shared<filters::FilterChain<sensor_msgs::msg::LaserScan>>("sensor_msgs::msg::LaserScan");

  // Configure filter chain
  filter_chain_->configure("", get_node_logging_interface(), get_node_parameters_interface());

  // Setup tf::MessageFilter for input
  tf_filter_->registerCallback(std::bind(&GenericLaserScanFilter::callback, this, std::placeholders::_1));
  tf_filter_->setTolerance(0.03s);

  // Advertise output
  output_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("output", 1000);

  std::function<void(const sensor_msgs::msg::LaserScan::SharedPtr)> standard_callback =
      std::bind(&GenericLaserScanFilter::foo, this, std::placeholders::_1);
  create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), standard_callback);

  deprecation_timer_ = create_wall_timer(5s, [this]() {
    RCLCPP_WARN(get_logger(), "'generic_laser_filter_node' has been deprecated. "
                              "Please switch to 'scan_to_scan_filter_chain'.");
  });
}

void GenericLaserScanFilter::callback(const std::shared_ptr<const sensor_msgs::msg::LaserScan> &msg_in)
{
  // Run the filter chain
  filter_chain_->update(*msg_in, msg_);

  // Publish the output
  output_pub_->publish(msg_);
}
}  // namespace laser_filters
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(laser_filters::GenericLaserScanFilter)