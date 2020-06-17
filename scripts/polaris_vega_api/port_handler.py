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
