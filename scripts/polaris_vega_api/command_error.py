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


##############################################################################
#                          API EXCEPTIONS
##############################################################################

class CommandError(Exception):

    def __init__(self, command, value):
        self.command = command
        self.value = value
        if value == '01':
            self.msg = 'Error ' + value + ' Unexpected RESET response'
        elif value == '07':
            self.msg = 'Error ' + value + ' Incorrect number of parameters. Command size = ' + str(len(command))
        elif value == '13':
            self.msg = 'Error ' + value + ' Hardware error: unable to read the SROM device.'
        elif value == '23':
            self.msg = 'Error ' + value + " Command parameter is out of range."
        elif value == '34':
            self.msg = 'Error ' + value + " User parameter does not exist."
        elif value == '40':
            self.msg = 'Error ' + value + ' Tool Definition File Error: This occurs if the CRC failed or the file ' \
                                          'format is invalid '
        elif value == '42':
            self.msg = 'Error ' + value + ' No device detected'
        else:
            self.msg = 'Error ' + value

    def __str__(self):
        return 'CommandError: ' + self.msg + ' when calling command ' + self.command
