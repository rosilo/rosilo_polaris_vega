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

#include <thread>
#include <chrono>
#include <memory>
#include <iostream>

#include "ros/ros.h"

#include "rosilo_polaris_vega/rosilo_polaris_vega_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "polaris_vega_interface_example");

    ros::NodeHandle node_handle;

    std::unique_ptr<rosilo::PolarisVegaInterface> polaris_vega_interface(new rosilo::PolarisVegaInterface(node_handle));
    ros::spinOnce();

    for(int i=0;i<10;i++)
    {
        if(polaris_vega_interface->is_initialized())
            polaris_vega_interface->get_tool_pose(1);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        ros::spinOnce();
    }

    return 0;
}
