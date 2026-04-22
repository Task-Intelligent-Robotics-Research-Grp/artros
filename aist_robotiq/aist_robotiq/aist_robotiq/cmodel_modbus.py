# Software License Agreement (BSD License)
#
# Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of National Institute of Advanced Industrial
#    Science and Technology (AIST) nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Toshio Ueshiba
#
import threading
from aist_robotiq.cmodel_base import CModelBase
from aist_robotiq_msgs.msg    import CModelStatus
from pymodbus.exceptions      import ModbusIOException
from pymodbus.client          import ModbusTcpClient, ModbusSerialClient
from pymodbus.payload         import BinaryPayloadBuilder, BinaryPayloadDecoder
from pymodbus.constants       import Endian

#########################################################################
#  class CModelModbusBase                                               #
#########################################################################
class CModelModbusBase(CModelBase):
    def __init__(self, name):
        super().__init__(name)

    def disconnect(self):
        if self._client:          # (self._client is defined in derived class)
            self._client.close()

    def put_command(self, command):
        # Clip each field of command within a valid range.
        command = self._clip_command(command)

        # Convert the command to a byte array of 6/15-length.
        builder = BinaryPayloadBuilder(byteorder=Endian.BIG,
                                       wordorder=Endian.BIG)
        builder.add_8bit_uint(command.r_act |
                              (command.r_mod << 1) |
                              (command.r_gto << 3) |
                              (command.r_atr << 4) |
                              (command.r_ard << 5))     # Byte 0
        builder.add_8bit_uint((command.r_icf << 2) |
                              (command.r_ics << 3))     # Byte 1
        builder.add_8bit_uint(0)                        # Byte 2
        builder.add_8bit_uint(command.r_pr)             # Byte 3
        builder.add_8bit_uint(command.r_sp)             # Byte 4
        builder.add_8bit_uint(command.r_fr)             # Byte 5
        if self._arg3f[command.r_sid]:
            builder.add_8bit_uint(command.r_prb)        # Byte 6
            builder.add_8bit_uint(command.r_spb)        # Byte 7
            builder.add_8bit_uint(command.r_frb)        # Byte 8
            builder.add_8bit_uint(command.r_prc)        # Byte 9
            builder.add_8bit_uint(command.r_spc)        # Byte 10
            builder.add_8bit_uint(command.r_frc)        # Byte 11
            builder.add_8bit_uint(command.r_prs)        # Byte 12
            builder.add_8bit_uint(command.r_sps)        # Byte 13
            builder.add_8bit_uint(command.r_frs)        # Byte 14
        self._write_registers(builder.to_registers(), command.r_sid)

    def get_status(self, slave_id):
        # Acquire status from the Gripper
        nregs = 8 if self._arg3f[slave_id] else 3
        try:
            result = self._read_registers(nregs, slave_id)
            if result.isError():
                self.get_logger().error(f'{result}')
                return None
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            return None

        decoder = BinaryPayloadDecoder.fromRegisters(result.registers,
                                                     byteorder=Endian.BIG,
                                                     wordorder=Endian.BIG)

        # Assign the values to their respective variables
        status = CModelStatus()
        status.g_sid = slave_id
        data = decoder.decode_8bit_uint()               # Byte 0
        status.g_act =  data       & 0x01
        status.g_mod = (data >> 1) & 0x03
        status.g_gto = (data >> 3) & 0x01
        status.g_sta = (data >> 4) & 0x03
        status.g_obj = (data >> 6) & 0x03
        data = decoder.decode_8bit_uint()               # Byte 1
        status.g_vas =  data       & 0x03
        status.g_dtb = (data >> 2) & 0x03
        status.g_dtc = (data >> 4) & 0x03
        status.g_dts = (data >> 6) & 0x03
        data = decoder.decode_8bit_uint()               # Byte 2
        status.g_flt = data        & 0x0f
        status.g_pr  = decoder.decode_8bit_uint()       # Byte 3
        status.g_po  = decoder.decode_8bit_uint()       # Byte 4
        status.g_cou = decoder.decode_8bit_uint()       # Byte 5
        if self._arg3f[slave_id]:
            status.g_prb = decoder.decode_8bit_uint()   # Byte 6
            status.g_pob = decoder.decode_8bit_uint()   # Byte 7
            status.g_cub = decoder.decode_8bit_uint()   # Byte 8
            status.g_prc = decoder.decode_8bit_uint()   # Byte 9
            status.g_poc = decoder.decode_8bit_uint()   # Byte 10
            status.g_cuc = decoder.decode_8bit_uint()   # Byte 11
            status.g_prs = decoder.decode_8bit_uint()   # Byte 12
            status.g_pos = decoder.decode_8bit_uint()   # Byte 13
            status.g_cus = decoder.decode_8bit_uint()   # Byte 14
        return status

#########################################################################
#  class CModelModbusTCP                                                #
#########################################################################
class CModelModbusTCP(CModelModbusBase):
    def __init__(self, name):
        super().__init__(name)
        ip = self.declare_parameter('ip', '192.168.1.11').value
        self._lock   = threading.Lock()
        self._client = ModbusTcpClient(ip)
        if not self._client.connect():
            self.get_logger().error('failed to connect[ip=%s]' % ip)
            raise
        self.get_logger().info('started[ip=%s]' % ip)

    def _write_registers(self, registers, slave_id):
        with self._lock:
            self._client.write_registers(0, registers, slave_id)

    def _read_registers(self, nregs, slave_id):
        with self._lock:
            return self._client.read_input_registers(0, nregs, slave_id)

#########################################################################
#  class CModelModbusRTU                                                #
#########################################################################
class CModelModbusRTU(CModelModbusBase):
    def __init__(self, name):
        super().__init__(name)
        dev = self.declare_parameter('dev', '/dev/ttyUSB0').value
        self._lock   = threading.Lock()
        self._client = ModbusSerialClient(port=dev,
                                          baudrate=115200, bytesize=8,
                                          parity='N', stopbits=1, timeout=1.0)
        if not self._client.connect():
            self.get_logger().error('failed to connect[dev=%s]' % dev)
            raise
        self.get_logger().info('started[dev=%s]' % dev)

    def _write_registers(self, registers, slave_id):
        with self._lock:
            self._client.write_registers(0x03E8, registers, slave_id)

    def _read_registers(self, nregs, slave_id):
        with self._lock:
            return self._client.read_input_registers(0x07D0, nregs, slave_id)
