"""
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
