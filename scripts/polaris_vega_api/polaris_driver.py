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

import binascii
import math
import struct

import serial

from polaris_vega_api.port_handle import PortHandle
from polaris_vega_api.port_handler import PortHandler
from polaris_vega_api.stray_marker import StrayMarker
from polaris_vega_api.translation import Translation
from polaris_vega_api.command_error import CommandError


def string_swap(str_to_swap):
    """Method used to swap the data endian"""
    size = int(len(str_to_swap) / 2)
    ret_str = ''
    for i in range(size):
        ret_str = str_to_swap[:2] + ret_str
        str_to_swap = str_to_swap[2:]
    return ret_str


def bytes_swap(bytes_to_swap):
    """Method used to swap the data endian"""
    size = int(len(bytes_to_swap) / 2)
    ret_bytes = b''
    for i in range(size):
        ret_bytes = bytes_to_swap[:2] + ret_bytes
        bytes_to_swap = bytes_to_swap[2:]
    return ret_bytes


def string_to_bytes(string):
    return str(string).encode('ascii')


def bytes_to_string(bytes_arg, encoding='ascii'):
    return bytes_arg.decode(encoding)


def crop_crc16_from_tail(response_as_string):
    cropped_response = response_as_string[:-4]
    return cropped_response


##############################################################################
#                            CLASS PolarisDriver
##############################################################################

class PolarisDriver:
    #######################
    #  API Constants
    #######################

    # PHSR Constants
    PHSR_REPORT_ALLOCATED = '00'
    PHSR_REPORT_NEED_FREE = '01'
    PHSR_REPORT_NOT_INIT = '02'
    PHSR_REPORT_NOT_ENABLED = '03'
    PHSR_REPORT_ENABLED = '04'

    # PENA Constants
    PENA_TTPRIO_STATIC = 'S'
    PENA_TTPRIO_DYNAMIC = 'D'
    PENA_TTPRIO_BBOX = 'B'

    # TSTART Constants
    # TSTART_RESET_FRAMECOUNT = '80' ignored in polaris Vega
    # TSTART_NORESET = '' unnecessary in polaris Vega

    #######################
    #  CONSTRUCTOR
    #######################

    def __init__(self):

        # PortHandler
        self.port_handler = PortHandler()

        self.serial = None
        self.debug = False
        self.frame_number = 0

    def initialize_serial_communication(self, port, timeout=0.3):

        # The other parameters need not be changed
        self.serial = serial.Serial()
        self.serial.port = port
        self.serial.timeout = timeout

    def initialize_ip_communication(self, ip, port):

        self.serial = serial.serial_for_url("socket://" + str(ip) + ":" + str(port))

    #######################
    #  PortHandler Functions
    #######################

    def init(self):
        return self._init()

    def set_fps(self, fps):
        if fps == 20:
            irate = 0
        elif fps == 30:
            irate = 1
        elif fps == 60:
            irate = 2
        else:
            raise Exception("Invalid fsp={}. It should be 20, 30, or 60".format(fps))

        self._irate(irate)
        reply_as_string = self._get("Param.Tracking.Track Frequency")
        if irate != int(reply_as_string[31:]):
            raise Exception("set_fps failed: irate={}!={}".format(int(reply_as_string[31:]), irate))

        self._set("Param.Tracking.Frame Frequency", str(fps))
        reply_as_string = self._get("Param.Tracking.Frame Frequency")
        if fps != int(reply_as_string[31:]):
            raise Exception("set_fps failed: fps={}!={}".format(int(reply_as_string[31:]), fps))

    def soft_reset(self):
        self._reset("0")

    def hard_reset(self):
        self._reset("1")

    def send_tool_definition(self, port_handle, file_name):
        piece_size = 64
        rom_file = open(str(file_name), "rb")
        address = 0
        print("Sending tool definition file {} for port handle: {} ...".format(file_name, port_handle))

        while True:
            # Convert address to necessary format
            converted_address = hex(address)[2:].zfill(4)
            if self.debug:
                print("Sending package to address {}: ".format(converted_address))

            # Gets data piece from file
            data_piece_binary_bytes = rom_file.read(piece_size)
            if data_piece_binary_bytes == b'':
                break
            if self.debug:
                print("data_piece_binary_bytes:[", len(data_piece_binary_bytes), "]", data_piece_binary_bytes)

            # Pads data if needed
            data_piece_length = len(data_piece_binary_bytes)
            if data_piece_length < piece_size:
                data_piece_binary_bytes += bytes(piece_size - data_piece_length)
            if self.debug:
                print("data_piece_binary_bytes:[", len(data_piece_binary_bytes), "]", data_piece_binary_bytes)

            data_piece_ascii_bytes = binascii.b2a_hex(data_piece_binary_bytes)
            if self.debug:
                print("data_piece_ascii_bytes:[", len(data_piece_ascii_bytes), "]", data_piece_ascii_bytes)

            command_as_string = bytes_to_string(data_piece_ascii_bytes)
            self._pvwr(port_handle, converted_address, command_as_string.upper())

            # Increase address
            address = address + piece_size

        rom_file.close()
        print("... Sending finished.")

    def add_passive_tool(self, rom_filename):
        ret = self._phrq("********", "*", "1", "00", "**")
        port_handle = ret[0:2]
        print("Acquired port handle: {}".format(port_handle))
        self.send_tool_definition(port_handle, rom_filename)
        self._pena(port_handle, self.PENA_TTPRIO_DYNAMIC)
        phinf_ret_as_string = crop_crc16_from_tail(self._phinf(port_handle, "0001"))
        port_status = phinf_ret_as_string[-2:]
        # Check if tool in port, port initialized, and port enabled
        if port_status != "31":
            raise Exception("Passive tool port handle not initialized correctly. {}!=31".format(port_status))
        self.port_handler.add_port_handle(PortHandle(port_handle, port_status))

    def assign_port_handle_all(self):
        self.port_handler = PortHandler(self._phsr(self.PHSR_REPORT_ALLOCATED))

    def init_port_handle_all(self):
        for handle in self.port_handler.handles:
            self._pinit(handle.id)
        self.assign_port_handle_all()

    def init_port_handle(self, port_handle):
        self._pinit(port_handle)

    def enable_port_handle(self, port_handle, ttpriority):
        return self._pena(port_handle.id, ttpriority)

    def start_tracking(self):
        return self._tstart()

    def stop_tracking(self):
        return self._tstop()

    def get_position_from_bx(self, reply_option):
        bx_data = self.bx(reply_option).encode('hex')
        # print bx_data
        bx_data = bx_data[42:66]

        def string_swap(str_to_swap):
            """Method used to swap the data endian"""
            size = len(str_to_swap) / 2
            ret_str = ''
            for i in range(size):
                ret_str = str_to_swap[:2] + ret_str
                str_to_swap = str_to_swap[2:]
            return ret_str

        x_str = bx_data[:8]
        bx_data = bx_data[8:]
        y_str = bx_data[:8]
        bx_data = bx_data[8:]
        z_str = bx_data[:8]
        bx_data = bx_data[8:]

        if len(x_str) > 7:
            x = struct.unpack("!f", string_swap(x_str[:8]).decode('hex'))[0]
        else:
            x = 'miss'

        if len(y_str) > 7:
            y = struct.unpack("!f", string_swap(y_str[:8]).decode('hex'))[0]
        else:
            y = 'miss'

        if len(z_str) > 7:
            z = struct.unpack("!f", string_swap(z_str[:8]).decode('hex'))[0]
        else:
            z = 'miss'

        pos = [x, y, z]
        print(pos)

        return pos

    def update_tool_transformations(self):

        raw_response_bytes = self._bx('1001')
        numhandles = int(raw_response_bytes[:2])
        raw_response_bytes = raw_response_bytes[2:]  # Crop the response bytes

        if numhandles != len(self.port_handler.handles):
            print(
                '''Reply has a different number of handles than expected! Reassigning ports should correct this 
                mistake. ''')
            return
        else:
            if self.debug:
                print('Number of handles: ', numhandles)
            for handle in self.port_handler.handles:

                # Check handle ID
                cur_handle_id = bytes_to_string(raw_response_bytes[:2])
                raw_response_bytes = raw_response_bytes[2:]  # Crop the response bytes
                if cur_handle_id != handle.id:
                    print('Unexpected handle id: {}!={}'.format(cur_handle_id, handle.id))

                # Check status
                cur_handle_status = bytes_to_string(raw_response_bytes[:2])
                raw_response_bytes = raw_response_bytes[2:]  # Crop the response bytes
                if cur_handle_status == handle.tool.TOOL_STATUS_VALID:

                    # Valid
                    handle.tool.status = handle.tool.TOOL_STATUS_VALID
                    if self.debug:
                        print('Tool with ID ' + str(handle.id) + ' is valid')

                    # Quaternion
                    q0 = struct.unpack("<f", binascii.unhexlify(raw_response_bytes[:8]))[0]
                    raw_response_bytes = raw_response_bytes[8:]
                    q1 = struct.unpack("<f", binascii.unhexlify(raw_response_bytes[:8]))[0]
                    raw_response_bytes = raw_response_bytes[8:]
                    q2 = struct.unpack("<f", binascii.unhexlify(raw_response_bytes[:8]))[0]
                    raw_response_bytes = raw_response_bytes[8:]
                    q3 = struct.unpack("<f", binascii.unhexlify(raw_response_bytes[:8]))[0]
                    raw_response_bytes = raw_response_bytes[8:]

                    handle.tool.update_rotation_quaternion(q0, q1, q2, q3)

                    # Translation
                    x = struct.unpack("<f", binascii.unhexlify(raw_response_bytes[:8]))[0] / 1000.0
                    raw_response_bytes = raw_response_bytes[8:]
                    # print raw_response_bytes
                    y = struct.unpack("<f", binascii.unhexlify(raw_response_bytes[:8]))[0] / 1000.0
                    raw_response_bytes = raw_response_bytes[8:]
                    # print raw_response_bytes
                    z = struct.unpack("<f", binascii.unhexlify(raw_response_bytes[:8]))[0] / 1000.0
                    # print raw_response_bytes
                    raw_response_bytes = raw_response_bytes[8:]

                    handle.tool.update_translation(x, y, z)

                    error = struct.unpack("<f", binascii.unhexlify(raw_response_bytes[:8]))[0]
                    raw_response_bytes = raw_response_bytes[8:]

                    handle.tool.update_error(error)

                    port_status = bytes_swap(raw_response_bytes[:8])
                    raw_response_bytes = raw_response_bytes[8:]

                    frame_number = struct.unpack("<I", binascii.unhexlify(raw_response_bytes[:8]))[0]
                    raw_response_bytes = raw_response_bytes[8:]

                    handle.tool.update_frame_number(frame_number)

                elif cur_handle_status == handle.tool.TOOL_STATUS_MISSING:
                    handle.tool.status = handle.tool.TOOL_STATUS_MISSING
                    if self.debug:
                        print('Tool with ID ' + str(handle.id) + ' is missing')
                    port_status = bytes_swap(raw_response_bytes[:8])
                    raw_response_bytes = raw_response_bytes[8:]
                    frame_number = struct.unpack("<f", binascii.unhexlify(raw_response_bytes[:8]))[0]
                    raw_response_bytes = raw_response_bytes[8:]

                elif cur_handle_status == handle.tool.TOOL_STATUS_DISABLED:
                    handle.tool.status = handle.tool.TOOL_STATUS_DISABLED
                    if self.debug:
                        print('Tool with ID ' + str(handle.id) + ' is disabled')

        # Stray markers
        self.straymarkerlist = []
        #
        numstraymarkers = int(raw_response_bytes[:2])
        raw_response_bytes = raw_response_bytes[2:]
        if self.debug:
            print('numstraymarkers : ', numstraymarkers)
            print('length of message: ', len(raw_response_bytes))
        for i in range(0, int(math.ceil(float(numstraymarkers) / 8.))):
            straymarkerstatus = int(raw_response_bytes[:2])
            raw_response_bytes = raw_response_bytes[2:]
            print("Straymarkerstatus ", straymarkerstatus)
            print('length of message: ', len(raw_response_bytes))
        if len(raw_response_bytes) > 10:
            # TODO don't ignore this info
            for i in range(0, numstraymarkers):
                # Translation
                x = struct.unpack("<f", binascii.unhexlify(raw_response_bytes[:8]))[0] / 1000.0
                raw_response_bytes = raw_response_bytes[8:]
                # print raw_response_bytes
                y = struct.unpack("<f", binascii.unhexlify(raw_response_bytes[:8]))[0] / 1000.0
                raw_response_bytes = raw_response_bytes[8:]
                # print raw_response_bytes
                z = struct.unpack("<f", binascii.unhexlify(raw_response_bytes[:8]))[0] / 1000.0
                # print raw_response_bytes
                raw_response_bytes = raw_response_bytes[8:]

                curstraymarker = StrayMarker(Translation(x, y, z), StrayMarker.STRAYMARKER_STATUS_UNDEFINED)
                self.straymarkerlist.append(curstraymarker)
                print('Marker ', i, ' is ', curstraymarker)
        else:
            self.straymarkerlist = []

    def get_tool(self, tool_index):
        if tool_index > len(self.port_handler.handles):
            raise Exception("Invalid tool index {} > {}".format(tool_index,
                                                                len(self.port_handler.handles)))
        return self.port_handler.handles[tool_index].tool

    def get_tools(self):
        tools_list = []
        for handle in self.port_handler.handles:
            tools_list.append(handle.tool)

        return tools_list

    #######################
    #  SERIAL COMM FUNCTIONS
    #######################

    def open(self):
        if self.serial.isOpen():
            print("port already opened.")
        else:
            self.serial.open()
        # self.serial.flushInput()
        # self.serial.flushOutput()

    def close(self):
        if self.serial.isOpen():
            self.serial.close()
        else:
            print('port already closed')

    def _serial_read_line(self):
        fullstring = ''
        while True:
            response_as_string = bytes_to_string(self.serial.read(1))
            if response_as_string == '\r':
                break
            fullstring = fullstring + response_as_string
        return fullstring

    def _send_command_and_check_response(self, command_as_string):
        command_as_bytes = string_to_bytes(command_as_string)
        self.serial.write(command_as_bytes)
        response_as_string = self._serial_read_line()
        if self.debug:
            print("Sent command [{}]: {}".format(len(command_as_string), command_as_string))
            print("Got reply [{}]: {}".format(len(response_as_string), response_as_string))
        self._raise_exception_if_error(command_as_string, response_as_string)
        return response_as_string

    #######################
    #  API COMMANDS
    #######################

    def _3d(self, port_handle, reply_option):
        command = '3D ' + str(port_handle) + str(reply_option) + '\r'
        return self._send_command_and_check_response(command)

    def _apirev(self):
        command = 'APIREV \r'
        return self._send_command_and_check_response(command)

    def get_api_rev(self):
        return self._apirev()

    def _beep(self, number_of_beeps):
        command = 'BEEP ' + str(number_of_beeps) + '\r'
        return self._send_command_and_check_response(command)

    def beep(self, number_of_beeps):
        self._beep(number_of_beeps)

    def _bx(self, reply_option):
        command_as_bytes = string_to_bytes('bx ' + str(reply_option) + '\r')

        # The reply to this command is binary so we need to decode it properly
        self.serial.write(command_as_bytes)

        start_sequence = binascii.hexlify(self.serial.read(2))
        if start_sequence != b'c4a5':
            print('Start sequence was {} but should be c4a5'.format(start_sequence))

        # Get reply length
        reply_length_string = bytes_to_string(binascii.hexlify(self.serial.read(2)))
        reply_length = int(string_swap(reply_length_string), 16)
        # Get CRC header
        header_crc = binascii.hexlify(self.serial.read(2))  # TODO check this CRC
        # Get Response
        raw_response_bytes = binascii.hexlify(self.serial.read(reply_length))
        # Get CRC
        crc = binascii.hexlify(self.serial.read(2))  # TODO check this CRC
        return raw_response_bytes

    def bx(self, reply_option):
        self.serial.write(string_to_bytes('bx ' + str(reply_option) + '\r'))

        start_sqn = self.serial.read(64)
        pos = self.serial.read(24)
        crc = self.serial.read(8)
        # print start_sqn + pos + crc
        return start_sqn + pos + crc

    def _comm(self, baud_rate, data_bits, parity, stop_bits, hardware_handshaking):
        self.serial.write(
            'COMM ' + str(baud_rate) + str(data_bits) + str(parity) + str(stop_bits) + str(hardware_handshaking) + '\r')
        return self._serial_read_line()

    def _dflt(self, user_parameter_name):
        self.serial.write('DFLT ' + str(user_parameter_name) + '\r')
        return self._serial_read_line()

    def _dstart(self, reply_option):
        self.serial.write('DSTART ' + str(reply_option) + '\r')
        return self._serial_read_line()

    def _dstop(self):
        self.serial.write('DSTOP \r')
        return self._serial_read_line()

    def _echo(self, ascii_characters):
        self.serial.write('DFLT ' + str(ascii_characters) + '\r')
        return self._serial_read_line()

    def _get(self, user_parameter_name):
        command_string = 'GET ' + str(user_parameter_name) + '\r'
        response_as_string = self._send_command_and_check_response(command_string)
        return crop_crc16_from_tail(response_as_string)

    def _getinfo(self, user_parameter_name):
        command_string = self.serial.write('GETINFO ' + str(user_parameter_name) + '\r')
        return self._send_command_and_check_response(command_string)

    def _getio(self):
        self.serial.write('GETIO \r')
        return self._serial_read_line()

    def _getlog(self, offset, length, logname):
        self.serial.write('GETLOG ' + str(offset) + str(length) + str(logname) + '\r')
        return self._serial_read_line()

    def _hcwdog(self, timeout_value):
        self.serial.write('HCWDOG ' + str(timeout_value) + '\r')
        return self._serial_read_line()

    def _init(self):
        command_as_string = 'INIT \r'
        return self._send_command_and_check_response(command_as_string)

    def _irate(self, illuminator_rate):
        command_as_string = 'IRATE ' + str(illuminator_rate) + '\r'
        return self._send_command_and_check_response(command_as_string)

    def _ired(self, port_handle, marker_activation_signature):
        self.serial.write('IRED ' + str(port_handle) + str(marker_activation_signature) + '\r')
        return self._serial_read_line()

    def _led(self, port_handle, led_number, state):
        self.serial.write('LED ' + str(port_handle) + str(led_number) + str(state) + '\r')
        return self._serial_read_line()

    def _pdis(self, port_handle):
        self.serial.write('PDIS ' + str(port_handle) + '\r')
        return self._serial_read_line()

    def _pena(self, port_handle, tool_tracking_priority):
        command_as_string = 'PENA ' + str(port_handle) + str(tool_tracking_priority) + '\r'
        return self._send_command_and_check_response(command_as_string)

    def _pfsel(self, port_handle, face_selection):
        self.serial.write('PFSEL ' + str(port_handle) + str(face_selection) + '\r')
        return self._serial_read_line()

    def _phf(self, port_handle):
        self.serial.write('PHF ' + str(port_handle) + '\r')
        return self._serial_read_line()

    def _phinf(self, port_handle, reply_option):
        command_as_string = 'PHINF ' + str(port_handle) + str(reply_option) + '\r'
        return self._send_command_and_check_response(command_as_string)

    def _phrq(self, hardware_device, system_type, tool_type, port_number, dummy_tool):
        command_as_string = 'PHRQ ' + str(hardware_device) + str(system_type) + str(tool_type) + str(port_number) + str(
            dummy_tool) + '\r'
        return self._send_command_and_check_response(command_as_string)

    def _phsr(self, reply_option):
        command_as_string = 'PHSR ' + str(reply_option) + '\r'
        return self._send_command_and_check_response(command_as_string)

    def _pinit(self, port_handle):
        command_as_string = 'PINIT ' + str(port_handle) + '\r'
        return self._send_command_and_check_response(command_as_string)

    def _pprd(self, port_handle, srom_device_address):
        self.serial.write('PPRD ' + str(port_handle) + str(srom_device_address) + '\r')
        return self._serial_read_line()

    def _ppwr(self, port_handle, srom_device_address, srom_device_data):
        self.serial.write('PPWR ' + str(port_handle) + str(srom_device_address) + str(srom_device_data) + '\r')
        return self._serial_read_line()

    def _psel(self, port_handle, srom_device_id):
        self.serial.write('PSEL ' + str(port_handle) + str(srom_device_id) + '\r')
        return self._serial_read_line()

    def _psout(self, port_handle, gpio_1_state, gpio_2_state, gpio_3_state, gpio_4_state):
        self.serial.write(
            'PSOUT ' + str(gpio_1_state) + str(gpio_2_state) + str(gpio_3_state) + str(gpio_4_state) + '\r')
        return self._serial_read_line()

    def _psrch(self, port_handle):
        self.serial.write('PSRCH ' + str(port_handle) + '\r')
        return self._serial_read_line()

    def _purd(self, port_handle, user_srom_device_address):
        self.serial.write('PURD ' + str(port_handle) + str(user_srom_device_address) + '\r')
        return self._serial_read_line()

    def _puwr(self, port_handle, user_srom_device_address, user_srom_device_data):
        self.serial.write(
            'PUWR ' + str(port_handle) + str(user_srom_device_address) + str(user_srom_device_data) + '\r')
        return self._serial_read_line()

    def _pvwr(self, port_handle, start_address, tool_definition_file_data):
        command_as_string = 'PVWR ' + str(port_handle) + str(start_address) + str(tool_definition_file_data) + '\r'
        return self._send_command_and_check_response(command_as_string)

    def _reset(self, reset_option):
        command_as_string = 'RESET ' + str(reset_option) + '\r'
        # The system's reply for RESET is a bit different so we dont use _send_command_and_check_response
        self.serial.write(string_to_bytes(command_as_string))
        return self._serial_read_line()

    def _save(self):
        self.serial.write('SAVE \r')
        return self._serial_read_line()

    def _sensel(self, option):
        self.serial.write('SENSEL ' + str(option) + '\r')
        return self._serial_read_line()

    def _set(self, user_parameter_name, value):
        command_string = 'SET ' + str(user_parameter_name) + '=' + str(value) + '\r'
        return self._send_command_and_check_response(command_string)

    def _setio(self, input_output_line_status):
        self.serial.write('SETIO ' + str(input_output_line_status) + '\r')
        return self._serial_read_line()

    def _sflist(self, reply_option):
        command_string = 'SFLIST ' + str(reply_option) + '\r'
        return self._send_command_and_check_response(command_string)

    def _sstat(self, reply_option):
        self.serial.write('SSTAT ' + str(reply_option) + '\r')
        return self._serial_read_line()

    def _syslog(self, device_name, category, message):
        self.serial.write('SYSLOG ' + str(device_name) + str(category) + '=' + str(message) + '\r')
        return self._serial_read_line()

    def _tctst(self, port_handle):
        self.serial.write('TCTST ' + str(port_handle) + '\r')
        return self._serial_read_line()

    def _tstart(self):
        command_as_string = 'TSTART ' + '\r'
        return self._send_command_and_check_response(command_as_string)

    def _tstop(self):
        command_as_string = 'TSTOP \r'
        return self._send_command_and_check_response(command_as_string)

    def _ttcfg(self, port_handle):
        self.serial.write('TTCFG ' + str(port_handle) + '\r')
        return self._serial_read_line()

    def _tx(self, reply_option):
        self.serial.write('TX ' + str(reply_option) + '\r')
        return self._serial_read_line()

    def _ver(self, reply_option):
        self.serial.write('VER ' + str(reply_option) + '\r')
        return self._serial_read_line()

    def _vget(self, row, sensor, frame_index, start_column, end_column, stride):
        self.serial.write(
            'VGET ' + str(row) + str(sensor) + str(frame_index) + str(start_column) + str(end_column) + str(
                stride) + '\r')
        return self._serial_read_line()

    def _vsel(self, volume_number):
        self.serial.write('VSEL ' + str(volume_number) + '\r')
        return self._serial_read_line()

    def _vsnap(self):
        self.serial.write('VSNAP \r')
        return self._serial_read_line()

    #######################
    #  ERROR AUX FUNCTIONS
    #######################

    def _raise_exception_if_error(self, command, response):
        if response[:5] == 'ERROR':
            error_code = response[5:7]
            raise CommandError(command, error_code)
        elif response[:5] == 'RESET':
            raise CommandError(command, '01')
        else:
            return

    #######################
    #  HIGHER LEVEL FUNCTION
    #######################

    def init_passive_marker_tracker(self, passive_file_list):
        for passive_file in passive_file_list:
            self.add_passive_tool(passive_file)

    def init_stray_marker_tracker(self):
        verbose = False
        if verbose:
            print(self._ver("4"))
            print(self._comm("7", "0", "0", "0", "1"))
            print(self._ver("5"))
            print(self._getinfo("Config.*"))
            print(self._get("Device.*"))
            print(self._init())
            print(self._phsr("00"))
            print(self._getinfo("Param.Tracking.*"))
            print(self._getinfo("Features.Firmware.Version"))
            print(self._getinfo("Info.Status.Alerts"))
            print(self._getinfo("Info.Status.New Alerts"))
            print(self._getinfo("Features.Hardware.Serial Number"))
            print(self._ver("4"))
            print(self._getinfo("Features.Tools.*"))
            print(self._sflist("03"))
            print(self._getinfo("Param.Tracking.Selected Volume"))
            print(self._getinfo("SCU-0.Info.Status.New Alerts"))
            print(self._getinfo("SCU-0.Info.Status.Alerts"))
            print(self._phinf("01", "0075"))
            print(self._getinfo("Info.Status.New Alerts"))
            print(self._getinfo("Info.Status.Alerts"))
            print(self._getinfo("STB-0.Info.Status.New Alerts"))
            print(self._getinfo("STB-0.Info.Status.Alerts"))

            print(self._tstart('80'))
            print(self.get_position_from_bx("1801"))
            print(self._tstop())

            print(self._set("PS-0.Param.Tracking.Illuminator Rate", "2"))
            print(self._phrq("********", "*", "1", "****"))
            print(self._pvwr("02", "0000",
                             "4E444900D2110000010000000000000100000000031480345A00000004000000040000000000403F000000000000000000000000000000000000000000000000"))
            print(self._pvwr("02", "0040",
                             "00002041000000000000000000000000000000000000000052B8E4417B14244200000000000000000000B04200000000AE4731C2CDCC21420000000000000000"))
            print(self._pvwr("02", "0080",
                             "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"))
            print(self._pvwr("02", "00C0",
                             "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"))
            print(self._pvwr("02", "0100",
                             "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000803F00000000"))
            print(self._pvwr("02", "0140",
                             "000000000000803F00000000000000000000803F00000000000000000000803F0000000000000000000000000000000000000000000000000000000000000000"))
            print(self._pvwr("02", "0180",
                             "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"))
            print(self._pvwr("02", "01C0",
                             "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"))
            print(self._pvwr("02", "0200",
                             "0000000000000000000000000000000000000000000000000000000000000000000000000000000000010203000000000000000000000000000000001F1F1F1F"))
            print(self._pvwr("02", "0240",
                             "090000004E4449000000000000000000383730303333390000000000000000000000000009010101010000000000000000000000000000000001010101000000"))
            print(self._pvwr("02", "0280",
                             "000000000000000000000000008000290000000000000000000080BF000000000000000000000000000000000000000000000000000000000000000000000000"))
            print(self._pvwr("02", "02C0",
                             "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"))
            print(self._pinit("02"))
            print(self._phinf("02", "0075"))
            print(self._tstart('80'))
            print(self.get_position_from_bx('1803'))
            print(self._tstop())

            print(self._pena("02", "D"))

            print(self._tstart('80'))
        else:
            print("Restarting system...")
            self._ver("4")
            self._comm("7", "0", "0", "0", "1")
            self._ver("5")
            self._getinfo("Config.*")
            self._get("Device.*")

            print("Starting system...")
            self._init()
            self._phsr("00")
            self._getinfo("Param.Tracking.*")
            self._getinfo("Features.Firmware.Version")
            self._getinfo("Info.Status.Alerts")
            self._getinfo("Info.Status.New Alerts")
            self._getinfo("Features.Hardware.Serial Number")
            self._ver("4")
            self._getinfo("Features.Tools.*")
            self._sflist("03")
            self._getinfo("Param.Tracking.Selected Volume")
            self._getinfo("SCU-0.Info.Status.New Alerts")
            self._getinfo("SCU-0.Info.Status.Alerts")
            self._phinf("01", "0075")
            self._getinfo("Info.Status.New Alerts")
            self._getinfo("Info.Status.Alerts")
            self._getinfo("STB-0.Info.Status.New Alerts")
            self._getinfo("STB-0.Info.Status.Alerts")

            print("Starting passive tool handle...")
            self._tstart('80')
            self.get_position_from_bx("1801")
            self._tstop()

            self._set("PS-0.Param.Tracking.Illuminator Rate", "2")
            self._phrq("********", "*", "1", "****")
            self._pvwr("02", "0000",
                       "4E444900D2110000010000000000000100000000031480345A00000004000000040000000000403F000000000000000000000000000000000000000000000000")
            self._pvwr("02", "0040",
                       "00002041000000000000000000000000000000000000000052B8E4417B14244200000000000000000000B04200000000AE4731C2CDCC21420000000000000000")
            self._pvwr("02", "0080",
                       "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000")
            self._pvwr("02", "00C0",
                       "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000")
            self._pvwr("02", "0100",
                       "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000803F00000000")
            self._pvwr("02", "0140",
                       "000000000000803F00000000000000000000803F00000000000000000000803F0000000000000000000000000000000000000000000000000000000000000000")
            self._pvwr("02", "0180",
                       "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000")
            self._pvwr("02", "01C0",
                       "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000")
            self._pvwr("02", "0200",
                       "0000000000000000000000000000000000000000000000000000000000000000000000000000000000010203000000000000000000000000000000001F1F1F1F")
            self._pvwr("02", "0240",
                       "090000004E4449000000000000000000383730303333390000000000000000000000000009010101010000000000000000000000000000000001010101000000")
            self._pvwr("02", "0280",
                       "000000000000000000000000008000290000000000000000000080BF000000000000000000000000000000000000000000000000000000000000000000000000")
            self._pvwr("02", "02C0",
                       "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000")
            self._pinit("02")
            self._phinf("02", "0075")

            print("Enabling tool...")
            self._tstart('80')
            self.get_position_from_bx('1803')
            self._tstop()

            self._pena("02", "D")

            print("Start!")
            self._tstart('80')

        return
