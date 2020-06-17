/*
# Copyright (c) 2012-2020 Murilo Marques Marinho
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

#include "rosilo_polaris_vega/rosilo_polaris_vega_interface.h"
#include "rosilo_conversions/rosilo_conversions.h"

namespace rosilo
{

PolarisVegaInterface::PolarisVegaInterface(ros::NodeHandle& node_handle):
    node_handle_(node_handle),
    initialized_(false)
{
    subscriber_tools_getpose_ = node_handle.subscribe("/polaris_vega_interface/tools/getpose", 1, &PolarisVegaInterface::tools_pose_callback, this);
}

void PolarisVegaInterface::tools_pose_callback(const rosilo_polaris_vega::ToolsPoseArray::ConstPtr &msg)
{
    //Set as initialized
    if(!initialized_)
        initialized_ = true;

    //Copy current message to member variable
    tools_pose_array_ = *msg;
}

DQ PolarisVegaInterface::get_tool_pose(const int &port_handle_id) const
{
    if(initialized_)
    {
        for(auto i=0; i < tools_pose_array_.port_handles.size(); i++)
        {
            if(tools_pose_array_.port_handles[i]==port_handle_id)
            {
                return geometry_msgs_pose_to_dq(tools_pose_array_.poses[i]);
            }
        }
        throw std::runtime_error("Invalid port_handle_id in get_tool_pose()");
    }
    else
    {
        throw std::runtime_error("Trying to get handle ids from uninitialized polaris spectra interface");
    }
}

PolarisToolStatus PolarisVegaInterface::get_tool_status(const int &port_handle_id) const
{
    if(initialized_)
    {
        for(auto i=0; i < tools_pose_array_.port_handles.size(); i++)
        {
            if(tools_pose_array_.port_handles[i]==port_handle_id)
            {
                return PolarisToolStatus(tools_pose_array_.statuses[i]);
            }
        }
        throw std::runtime_error("Invalid port_handle_id in get_tool_pose()");
    }
    else
    {
        throw std::runtime_error("Trying to get handle ids from uninitialized polaris spectra interface");
    }
}

std::vector<int> PolarisVegaInterface::get_port_handle_ids() const
{
    if(initialized_)
    {
        return tools_pose_array_.port_handles;
    }
    else
    {
        throw std::runtime_error("Trying to get handle ids from uninitialized polaris spectra interface");
    }
}

bool PolarisVegaInterface::is_initialized() const
{
    return initialized_;
}

}
