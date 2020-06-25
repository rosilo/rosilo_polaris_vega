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


class Quaternion:
    def __init__(self, q0=0.0, q1=0.0, q2=0.0, q3=0.0):
        self.q0 = q0
        self.q1 = q1
        self.q2 = q2
        self.q3 = q3

    def __str__(self):
        return 'Quaternion = [' + str(self.q0) + ',' + str(self.q1) + ',' + str(self.q2) + ',' + str(self.q3) + ']'
