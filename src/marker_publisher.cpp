/*
 * Copyright (c) 2018, Houston Mechatronics Inc., JD Yamokoski
 * Copyright (c) 2012, Clearpath Robotics, Inc., Alex Bencz
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <mocap_optitrack/Marker.h>
#include <mocap_optitrack/MarkerInfo.h>
#include <mocap_optitrack/Markers.h>
#include <mocap_optitrack/marker_publisher.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <vector>

namespace mocap_optitrack
{

  namespace utilities
  {
    geometry_msgs::Point getRosPoint(data::Marker const &marker, const Version &coordinatesVersion)
    {
      geometry_msgs::Point point;
      if (coordinatesVersion < Version("2.0") && coordinatesVersion >= Version("1.7"))
      {
        // Motive 1.7+ and < Motive 2.0 coordinate system
        point.x = -marker.point.x;
        point.y = marker.point.z;
        point.z = marker.point.y;
      }
      else
      {
        // y & z axes are swapped in the Optitrack coordinate system
        // Also compatible with versions > Motive 2.0
        point.x = marker.point.x;
        point.y = -marker.point.z;
        point.z = marker.point.y;
      }
      return point;
    }
  } // namespace utilities

  MarkerPublisher::MarkerPublisher(ros::NodeHandle &nh,
                                                     Version const &natNetVersion,
                                                     PublisherConfiguration const &config) : config(config)
  {
    if (config.publishPoint)
    {
      markerPublisher = nh.advertise<mocap_optitrack::Markers>(config.markerTopicName, 1000);
      ROS_INFO_STREAM("Publishing on topic " << config.markerTopicName);
    }

    // Motive 1.7+ uses a new coordinate system
    // natNetVersion = (natNetVersion >= Version("1.7"));
    coordinatesVersion = natNetVersion;
  }

  MarkerPublisher::~MarkerPublisher()
  {
  }

  void MarkerPublisher::publish(ros::Time const &time, std::vector<data::Marker> const &markers)
  {
    mocap_optitrack::Markers markers_msg;
    for (auto &&marker : markers)
    {
      // NaN?
      if (marker.point.x != marker.point.x)
      {
        ROS_WARN("Marker contains NaN");
        continue;
      }

      mocap_optitrack::Marker marker_msg;
      marker_msg.point = utilities::getRosPoint(marker, coordinatesVersion);

      marker_msg.info.markerId = marker.info.markerId;
      marker_msg.info.modelId = marker.info.modelId;
      marker_msg.info.size = marker.info.active;
      marker_msg.info.occluded = marker.info.active;
      marker_msg.info.pcSolved = marker.info.active;
      marker_msg.info.modelSolved = marker.info.active;
      marker_msg.info.hasModel = marker.info.active;
      marker_msg.info.unlabeled = marker.info.active;
      marker_msg.info.active = marker.info.active;
      marker_msg.info.residual = marker.info.residual;

      markers_msg.markers.push_back(marker_msg);
    }
    markers_msg.header.stamp = time;
    markers_msg.header.frame_id = config.parentFrameId;
    markerPublisher.publish(markers_msg);
  }
} // namespace mocap_optitrack
