# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
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
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
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
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$
#
# Modifed from the orginal comModbusTcp by Kelsey Hawkins @ Georgia Tech


"""@package docstring
Module comModbusRtu: defines a class which communicates with Robotiq Grippers using the Modbus RTU protocol.

The module depends on pymodbus (http://code.google.com/p/pymodbus/) for the Modbus RTU client.
"""

from __future__ import print_function

from pymodbus.register_read_message import ReadHoldingRegistersResponse
from math import ceil
import time

import pymodbus.client as ModbusClient
from pymodbus import (
    ExceptionResponse,
    Framer,
    ModbusException,
    pymodbus_apply_logging_config,
)

class communication:

    def __init__(self):
        self.client = None

    def connectToDevice(self, device):
        """Connection to the client - the method takes the IP address (as a string, e.g. '192.168.1.11') as an argument."""
        self.client = ModbusClient.ModbusSerialClient(
        device,
        framer=Framer.RTU,
        timeout=3,
        retries=3,
        baudrate=115200,
        bytesize=8,
        parity="N",
        stopbits=1,
        # handle_local_echo=False,
    )
        if not self.client.connect():
            print("Unable to connect to {}".format(device))
            return False
        return True

    def disconnectFromDevice(self):
        """Close connection"""
        self.client.close()

    def sendCommand(self, data):
        """Send a command to the Gripper - the method takes a list of uint8 as an argument. The meaning of each variable depends on the Gripper model (see support.robotiq.com for more details)"""

        if not self.client.connect():
            print("Unable to connect to {}".format(self.client.comm_params.host))
            return
    
        # make sure data has an even number of elements
        if len(data) % 2 == 1:
            data.append(0)

        # Initiate message as an empty list
        message = []

        # Fill message by combining two bytes in one register
        for i in range(0, len(data) // 2):
            message.append((data[2 * i] << 8) + data[2 * i + 1])

        # To do!: Implement try/except
        self.client.write_registers(0x03E8, message, slave=0x0009)

    def getStatus(self, numBytes):
        """Sends a request to read, wait for the response and returns the Gripper status. The method gets the number of bytes to read as an argument"""
        # Instantiate output as an empty list
        output = []

        if not self.client.connect():
            print("Unable to connect to {}".format(self.client.comm_params.host))
            return output
        
        numRegs = int(ceil(numBytes / 2.0))

        # To do!: Implement try/except
        # Get status from the device
        try:
            # See all calls in client_calls.py
            rr = self.client.read_holding_registers(0x07D0, count=numRegs, slave=0x0009)
        except ModbusException as exc:
            print(f"Received ModbusException({exc}) from library")
            self.client.close()
            return output

        if rr.isError():
            print(f"Received exception from device ({rr})")
            # THIS IS NOT A PYTHON EXCEPTION, but a valid modbus message
            self.client.close()
            return output

        # print(f"Got response: {type(rr)}")
        # retries = 50
        # # To get around spuratic - object has no attribute 'registers'
        # while not isinstance(rr, ReadHoldingRegistersResponse) and retries > 0:
        #     rr = self.client.read_holding_registers(0x07D0, numRegs, slave=0x0009)
        #     retries -= 1
        #     time.sleep(0.01)

        if isinstance(rr, ReadHoldingRegistersResponse):
            # Fill the output with the bytes in the appropriate order
            for i in range(0, numRegs):
                output.append((rr.getRegister(i) & 0xFF00) >> 8)
                output.append(rr.getRegister(i) & 0x00FF)

        # Output the result
        return output
