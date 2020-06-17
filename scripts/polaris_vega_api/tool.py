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

from polaris_vega_api.translation import Translation
from polaris_vega_api.quaternion import Quaternion


class Tool:
    TOOL_STATUS_VALID = '01'
    TOOL_STATUS_MISSING = '02'
    TOOL_STATUS_DISABLED = '04'

    def __init__(self, parent_port_handle=None):
        self.trans = Translation()
        self.rot = Quaternion()
        self.status = self.TOOL_STATUS_DISABLED
        self.error = 0.0
        self.frame_number = 0
        self.parent_port_handle = parent_port_handle

    def get_frame_number(self):
        return self.frame_number

    def get_parent_port_handle_id(self):
        return self.parent_port_handle.id

    def get_status(self):
        return self.status

    def get_status_string(self):
        if self.status == self.TOOL_STATUS_VALID:
            return "Valid"
        elif self.status == self.TOOL_STATUS_MISSING:
            return "Missing"
        elif self.status == self.TOOL_STATUS_DISABLED:
            return "Disabled"
        else:
            raise Exception("Unknown tool status")

    def update_frame_number(self, frame_number):
        self.frame_number = frame_number

    def update_error(self, error):
        self.error = error

    def update_rotation_quaternion(self, q0, q1, q2, q3):
        self.rot.q0 = q0
        self.rot.q1 = q1
        self.rot.q2 = q2
        self.rot.q3 = q3

    def update_translation(self, x, y, z):
        self.trans.x = x
        self.trans.y = y
        self.trans.z = z
