#!/usr/bin/env python3
import rospy
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse
from pymodbus_client.srv import mir_check, mir_checkResponse

def mir_callback(req):
    client = ModbusTcpClient('192.168.1.202', 5020)

    x = client.read_holding_registers(0,1)
    y = client.read_holding_registers(1,1)
    client.close()

    srv = mir_checkResponse()
    srv.x = x.registers[0]
    srv.y = y.registers[0]
    return srv

def mir_server():
    rospy.init_node('modbus_client')
    s = rospy.Service('mir_check', mir_check, mir_callback)
    print("Ready")
    rospy.spin()

if __name__ == "__main__":
    mir_server()
