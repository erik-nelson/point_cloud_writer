/*
 * Copyright (c) 2016, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
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
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: Erik Nelson            ( eanelson@eecs.berkeley.edu )
 */

#include <point_cloud_writer/PointCloudWriter.h>
#include <parameter_utils/ParameterUtils.h>

#include <pcl/io/ply_io.h>

namespace pu = parameter_utils;

PointCloudWriter::PointCloudWriter() {}
PointCloudWriter::~PointCloudWriter() {}

bool PointCloudWriter::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "PointCloudWriter");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  return true;
}

bool PointCloudWriter::LoadParameters(const ros::NodeHandle& n) {
  // Load output file parameters.
  if (!pu::Get("output_file", params_.output_file)) return false;
  if (!pu::Get("output_path", params_.output_path)) return false;

  return true;
}

bool PointCloudWriter::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create a local nodehandle to manage callback subscriptions.
  ros::NodeHandle nl(n);

  cloud_sub_ = nl.subscribe("point_cloud", 100,
                            &PointCloudWriter::PointCloudCallback, this);

  return true;
}

void PointCloudWriter::PointCloudCallback(const PointCloud::ConstPtr& msg) {
  Write(msg);
}

bool PointCloudWriter::Write(const PointCloud::ConstPtr& points) const {
  if (points == NULL) {
    ROS_ERROR("%s: Input is null.", name_.c_str());
    return false;
  }

  // Write the point cloud.
  const std::string filename = params_.output_path + "/" + params_.output_file;
  pcl::PLYWriter writer;
  writer.write<pcl::PointXYZ>(filename, *points, false);
  ROS_INFO("%s: Saved point cloud with %lu points to file: %s.", name_.c_str(),
           points->size(), filename.c_str());
  return true;
}
