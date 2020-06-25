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

from polaris_vega_api.port_handle import PortHandle


class PortHandler:

    def __init__(self, string=None):

        # Creating object with no arguments
        if string is None:
            self.numhandles = 0
            self.handles = []
            return

        self.numhandles = int(string[0:2])
        self.handles = []
        self.numstraymarkers = 0
        self.straymarkerlist = []

        # Iterate and add handles
        for i in range(self.numhandles):
            port_id = string[i * 5 + 2: i * 5 + 4]
            port_status = int(string[i * 5 + 4: i * 5 + 6])
            handle = PortHandle(port_id, port_status)
            self.handles.append(handle)

    def add_port_handle(self, port_handle):
        self.handles.append(port_handle)
        self.numhandles = len(self.handles)

    def __str__(self):
        print('Number of Handles: ', self.numhandles)
        for handle in self.handles:
            print(handle)
        return ''
