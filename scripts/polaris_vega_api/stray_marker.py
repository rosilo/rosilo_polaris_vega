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
