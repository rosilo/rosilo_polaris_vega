#pragma once
/*
# Copyright (c) 2012-2020 Murilo Marques Marinho
#
#    This file is part of rosilo_polaris_vega.
#
#    rosilo_polaris_vega is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    rosilo_polaris_vega is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with rosilo_polaris_vega.  If not, see <https://www.gnu.org/licenses/>.
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

std::string to_string(const PolarisToolStatus& status);

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

