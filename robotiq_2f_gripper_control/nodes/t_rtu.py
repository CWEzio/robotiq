#!/usr/bin/env python3
import pymodbus.client as ModbusClient
from pymodbus import (
    ExceptionResponse,
    Framer,
    ModbusException,
    pymodbus_apply_logging_config,
)


from pymodbus.register_read_message import ReadHoldingRegistersResponse

def run_sync_simple_client(port='/tmp/ttyUR16'):
    """Run sync client."""
    # activate debugging
    pymodbus_apply_logging_config("DEBUG")

    print("get client")
    client = ModbusClient.ModbusSerialClient(
        port,
        framer=Framer.RTU,
        timeout=3,
        retries=3,
        baudrate=115200,
        bytesize=8,
        parity="N",
        stopbits=1,
        # handle_local_echo=False,
    )


    print("connect to server")
    if not client.connect():
        print("Unable to connect to server")
        return

    print("get and verify data")

    try:
        # See all calls in client_calls.py
        rr = client.read_holding_registers(0x07D0, count=6, slave=0x0009)
    except ModbusException as exc:
        print(f"Received ModbusException({exc}) from library")
        client.close()
        return
    if rr.isError():
        print(f"Received exception from device ({rr})")
        # THIS IS NOT A PYTHON EXCEPTION, but a valid modbus message
        client.close()
        return
    
    print(f"Got response: {rr}")
    output=[]
    if isinstance(rr, ReadHoldingRegistersResponse):
        # Fill the output with the bytes in the appropriate order
        for i in range(0, 6):
            output.append((rr.getRegister(i) & 0xFF00) >> 8)
            output.append(rr.getRegister(i) & 0x00FF)
    print(f"Got output: {output}")
    value_int32 = client.convert_from_registers(rr.registers, data_type=client.DATATYPE.INT32)
    print(f"Got int32: {value_int32}")

    print("close connection")
    client.close()


if __name__ == "__main__":
    run_sync_simple_client()