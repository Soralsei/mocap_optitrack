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
#include <mocap_optitrack/unlabeled_marker_publisher.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <mocap_optitrack/Markers.h>
#include <vector>

namespace mocap_optitrack
{

  namespace utilities
  {
    geometry_msgs::PointStamped getRosPoint(Marker const &marker, const Version &coordinatesVersion)
    {
      geometry_msgs::PointStamped pointStamped;
      if (coordinatesVersion < Version("2.0") && coordinatesVersion >= Version("1.7"))
      {
        // Motive 1.7+ and < Motive 2.0 coordinate system
        pointStamped.point.x = -marker.x;
        pointStamped.point.y = marker.z;
        pointStamped.point.z = marker.y;
      }
      else
      {
        // y & z axes are swapped in the Optitrack coordinate system
        // Also compatible with versions > Motive 2.0
        pointStamped.point.x = marker.x;
        pointStamped.point.y = -marker.z;
        pointStamped.point.z = marker.y;
      }
      return pointStamped;
    }
  } // namespace utilities

  UnlabeledMarkerPublisher::UnlabeledMarkerPublisher(ros::NodeHandle &nh,
                                                     Version const &natNetVersion,
                                                     PublisherConfiguration const &config) : config(config)
  {
    if (config.publishPoint)
      markerPublisher = nh.advertise<geometry_msgs::PointStamped>(config.pointTopicName, 1000);

    // Motive 1.7+ uses a new coordinate system
    // natNetVersion = (natNetVersion >= Version("1.7"));
    coordinatesVersion = natNetVersion;
  }

  UnlabeledMarkerPublisher::~UnlabeledMarkerPublisher()
  {
  }

  void UnlabeledMarkerPublisher::publish(ros::Time const &time, std::vector<Marker> const &markers)
  {
    std::vector<geometry_msgs::PointStamped> points;
    for (auto &&marker : markers)
    {
      // NaN?
      if (marker.x != marker.x)
      {
        ROS_WARN("Marker contains NaN");
        continue;
      }

      geometry_msgs::PointStamped point = utilities::getRosPoint(marker, coordinatesVersion);

      point.header.stamp = time;
      point.header.frame_id = config.parentFrameId;
      points.push_back(point);
    }
    mocap_optitrack::Markers m;
    m.markers = points;
    markerPublisher.publish(m);
  }
} // namespace mocap_optitrack
