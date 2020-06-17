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


class StrayMarker:
    STRAYMARKER_STATUS_VALID = '01'
    STRAYMARKER_STATUS_OUTOFVOLUME = '02'
    STRAYMARKER_STATUS_UNDEFINED = 'UD'

    def __init__(self, trans, status):
        self.trans = trans
        self.status = self.STRAYMARKER_STATUS_UNDEFINED

    def updateStatus(self, status):
        self.status = status

    def updateTrans(self, x, y, z):
        self.trans.x = x
        self.trans.y = y
        self.trans.z = z

    def __str__(self):
        return 'Marker status: ' + str(self.status) + '\n   ' + self.trans.__str__()
