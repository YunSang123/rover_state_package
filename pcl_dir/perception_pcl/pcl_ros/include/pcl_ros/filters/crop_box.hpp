/*
 *
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  $Id: cropbox.cpp
 *
 */

#ifndef PCL_ROS__FILTERS__CROP_BOX_HPP_
#define PCL_ROS__FILTERS__CROP_BOX_HPP_

// PCL includes
#include <pcl/filters/crop_box.h>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include "pcl_ros/filters/filter.hpp"

namespace pcl_ros
{
/** \brief @b CropBox is a filter that allows the user to filter all the data inside of a given box.
  *
  * \author Radu Bogdan Rusu
  * \author Justin Rosen
  * \author Marti Morta Garriga
  */
class CropBox : public Filter
{
protected:
  /** \brief Call the actual filter.
    * \param input the input point cloud dataset
    * \param indices the input set of indices to use from \a input
    * \param output the resultant filtered dataset
    */
  inline void
  filter(
    const PointCloud2::ConstSharedPtr & input, const IndicesPtr & indices,
    PointCloud2 & output) override;

  /** \brief Parameter callback
    * \param params parameter values to set
    */
  rcl_interfaces::msg::SetParametersResult
  config_callback(const std::vector<rclcpp::Parameter> & params);
  /** \brief Create publishers for output PointCloud2 data as well as the crop box marker. */
  void createPublishers() override;
  /** \brief Update the crop box marker msg. */
  void update_marker_msg();

  OnSetParametersCallbackHandle::SharedPtr callback_handle_;
  /** \brief The crop box marker msg publisher for visualization and debugging purposes. */
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr crop_box_marker_publisher_;
  /**
   * \brief The crop box marker message.
   *
   * The marker's cube gets updated whenever the min/max points are changed.
   * The header is adjusted with every point cloud callback.
   */
  visualization_msgs::msg::Marker crop_box_marker_msg_;

private:
  /** \brief The PCL filter implementation used. */
  pcl::CropBox<pcl::PCLPointCloud2> impl_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit CropBox(const rclcpp::NodeOptions & options);
};
}  // namespace pcl_ros

#endif  // PCL_ROS__FILTERS__CROP_BOX_HPP_
