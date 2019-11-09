#!/usr/bin/env python3
import rospy
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse
from pymodbus_client.srv import feeder_srv

def feeder_check(req):
    result = False
    client = ModbusTcpClient('localhost', 5020)
    if req.colour == "YELLOW":
        coil = client.read_coils(0,1)
        result = coil.bits[0]
    elif req.colour == "RED":
        coil = client.read_coils(1,1)
        result = coil.bits[0]
    elif req.colour == "BLUE":
        coil = client.read_coils(2,1)
        result = coil.bits[0]

    client.close()
    return result

def feeder_server():
    rospy.init_node('modbus_client')
    s = rospy.Service('feeder_check', feeder_srv, feeder_check)
    print("Ready")
    rospy.spin()

if __name__ == "__main__":
    feeder_server()
