#!/usr/bin/python3

"""
# Copyright (c) 2012-2020
# Murilo Marques Marinho
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     - Neither the name of the software nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
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
"""

##############################################################################
#                INCLUDES
##############################################################################
# pip libraries
import yaml

# ROS
import roslib

roslib.load_manifest('polaris_vega_interface')
import rospy
import rospkg

# ROS Messages
from geometry_msgs.msg import Pose as rosmsg_Pose
from geometry_msgs.msg import Quaternion as rosmsg_Quaternion
from geometry_msgs.msg import Point as rosmsg_Point
from polaris_vega_interface.msg import ToolsPoseArray as rosmsg_ToolsPoseArray

# Polaris driver
from polaris_vega_api import PolarisDriver, Tool

##############################################################################
#              NODE MAIN ROUTINE
##############################################################################

def main():
    # Initialize Node. NOTE: NO INTERRUPT SIGNAL BY DEFAULT
    rospy.init_node(name='polaris_driver_node', disable_signals=True)

    # Initialize EMGSensor object
    polaris_driver_ros = PolarisDriverROS()

    # Loop
    polaris_driver_ros.loop()


class PolarisDriverROS:

    #################################
    #  CONSTRUCTORS AND DESTRUCTORS
    #################################

    def __init__(self):

        # Get package configuration file
        rospack = rospkg.RosPack()
        polaris_vega_interface_package_path = rospack.get_path('polaris_vega_interface')
        yaml_file = open(polaris_vega_interface_package_path + "/cfg/config.yml")
        parsed_yaml_file = yaml.load(yaml_file, Loader=yaml.FullLoader)

        # Initialize passive tools publisher
        self.pub_tools_pose = rospy.Publisher("/polaris_vega_interface/tools/getpose",
                                              rosmsg_ToolsPoseArray,
                                              queue_size=1)

        # Initialize polaris driver
        self.polaris_driver = PolarisDriver()
        self.polaris_driver.debug = parsed_yaml_file['polaris_vega']['debug']
        self.polaris_driver.initialize_ip_communication(parsed_yaml_file['polaris_vega']['ip'],
                                                        parsed_yaml_file['polaris_vega']['port'], )
        self.polaris_driver.open()
        self.polaris_driver.init()

        passive_file_list = []
        if 'passive_tools' in parsed_yaml_file and len(parsed_yaml_file['passive_tools']) > 0:
            for tool in parsed_yaml_file['passive_tools']:
                passive_file_list.append(tool['path'])
        else:
            raise Exception('Fatal error: Configuration file has no passive tool definitions.')

        self.polaris_driver.init_passive_marker_tracker(passive_file_list)

        self.polaris_driver.set_fps(parsed_yaml_file['polaris_vega']['fps'])

        self.polaris_driver.start_tracking()

    def close(self):
        self.polaris_driver.stop_tracking()
        self.polaris_driver.close()

    #################################
    # CLASS MAIN LOOP
    #################################

    def loop(self):

        while not rospy.is_shutdown():

            try:
                self.polaris_driver.update_tool_transformations()
                tools_pose_array = rosmsg_ToolsPoseArray()
                tools_pose_array.header.stamp = rospy.Time.now()
                tools_pose_array.header.frame_id = 'polaris_vega'

                for tool in self.polaris_driver.get_tools():
                    tool_pose = rosmsg_Pose()
                    tool_pose.orientation = rosmsg_Quaternion(tool.rot.q1, tool.rot.q2, tool.rot.q3, tool.rot.q0)
                    tool_pose.position = rosmsg_Point(tool.trans.x, tool.trans.y, tool.trans.z)

                    tools_pose_array.frame_numbers.append(int(tool.get_frame_number()))
                    tools_pose_array.port_handles.append(int(tool.get_parent_port_handle_id()))
                    tools_pose_array.statuses.append(int(tool.get_status()))
                    tools_pose_array.poses.append(tool_pose)

                self.pub_tools_pose.publish(tools_pose_array)

            except KeyboardInterrupt:
                print('Keyboard interrupt detected, shutting down node...')

                # Send stop tracking signal to Polaris and close serial port
                self.close()

                # Signal shutdown to node
                rospy.signal_shutdown('Keyboard interrupt')


##############################################################################
#              RUNNING THE MAIN ROUTINE
############################################################################## 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
