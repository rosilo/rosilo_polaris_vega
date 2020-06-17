#pragma once
/*
# Copyright (c) 2016-2020 Murilo Marques Marinho
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     - Neither the name of the Mitsuishi Sugita Laboratory (NML) nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################*/

#include <vector>
#include <atomic>

#include <dqrobotics/DQ.h>

#include <ros/ros.h>
#include <rosilo_polaris_vega/ToolsPoseArray.h>

using namespace DQ_robotics;

namespace rosilo
{

enum class PolarisToolStatus
{
    VALID = 1,
    MISSING = 2,
    DISABLED = 4
};

class PolarisVegaInterface
{
private:
    ros::NodeHandle node_handle_;
    std::atomic_bool initialized_;
    rosilo_polaris_vega::ToolsPoseArray tools_pose_array_;
    ros::Subscriber subscriber_tools_getpose_;

    void tools_pose_callback(const rosilo_polaris_vega::ToolsPoseArray::ConstPtr& msg);

public:
    PolarisVegaInterface()=delete;
    PolarisVegaInterface(ros::NodeHandle& node_handle);

    DQ get_tool_pose(const int& port_handle_id) const;
    PolarisToolStatus get_tool_status(const int& port_handle_id) const;
    std::vector<int> get_port_handle_ids() const;

    bool is_initialized() const;

};

}

