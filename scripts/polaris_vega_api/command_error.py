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
