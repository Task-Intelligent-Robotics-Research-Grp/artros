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

        # Convert the command to a byte array of 6-length.
        self.data = []
        self.data.append(command.r_act +
                         (command.r_mod << 1) + (command.r_gto << 3) +
                         (command.r_atr << 4) + (command.r_ard << 5))  # Byte0
        self.data.append(0)                                            # Byte1
        self.data.append(0)                                            # Byte2
        self.data.append(command.r_pr)                                 # Byte3
        self.data.append(command.r_sp)                                 # Byte4
        self.data.append(command.r_fr)                                 # Byte5
        self._put_command(self.data, command.r_sid)

    def get_status(self, slave_id):
        # Acquire status from the Gripper
        data = self._get_status(6, slave_id)

        # Assign the values to their respective variables
        status = CModelStatus()
        status.g_sid = slave_id
        status.g_act =  data[0]       & 0x01
        status.g_mod = (data[0] >> 1) & 0x03
        status.g_gto = (data[0] >> 3) & 0x01
        status.g_sta = (data[0] >> 4) & 0x03
        status.g_obj = (data[0] >> 6) & 0x03
        status.g_bas =  data[1]       & 0x03
        status.g_flt =  data[2]       & 0x0f
        status.g_pr  =  data[3]
        status.g_po  =  data[4]
        status.g_cou =  data[5]
        return status

    def _put_command(self, data, slave_id):
        # Make sure data has an even number of elements
        if len(data) % 2 == 1:
            data.append(0)

        # Compose every two bytes into one register word in big-endian order.
        message = []
        for i in range(0, len(data), 2):
            message.append((data[i] << 8) + data[i+1])
        self._write_registers(message, slave_id)  # (defined in derived class)

    def _get_status(self, nbytes, slave_id):
        nregs    = 2*((nbytes - 1)/2)
        response = self._read_registers(nregs,
                                        slave_id) # (defined in derived class)

        if isinstance(response, ModbusIOException):
            raise RuntimeError(response)

        # Decompose each register word to two bytes in little-endian order.
        data = []
        for val in response.registers:
            data.append((val & 0xFF00) >> 8)
            data.append( val & 0x00FF)
        return data

#########################################################################
#  class CModelModbusTCP                                                #
#########################################################################
class CModelModbusTCP(CModelModbusBase):
    def __init__(self, name):
        super().__init__(name)
        ip = self.declare_parameter('ip', '10.66.171.40').value
        self._lock   = threading.Lock()
        self._client = ModbusTcpClient(ip)
        self._client.connect()
        self.get_logger().info('started[ip=%s]' % ip)

    def _write_registers(self, message, slave_id):
        with self._lock:
            self._client.write_registers(0, message, slave_id)

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
        self._client = ModbusSerialClient(method='rtu', port=dev,
                                          stopbits=1, bytesize=8, parity='N',
                                          baudrate=115200, timeout=0.2)
        self._client.connect()
        self.get_logger().info('started[dev=%d]' % dev)

    def _write_registers(self, message, slave_id):
        with self._lock:
            self._client.write_registers(0x03E8, message, slave_id)

    def _read_registers(self, nregs, slave_id):
        with self._lock:
            return self._client.read_input_registers(0x07D0, nregs, slave_id)
