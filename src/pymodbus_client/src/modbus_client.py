#!/usr/bin/env python
import rospy
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse
from pymodbus_client.srv import mir_check, mir_checkResponse, feeder_srv, feeder_srvResponse


def feeder_check(req):
    srv = feeder_srvResponse()
    try:
        client = ModbusTcpClient('192.168.1.202', 5020)
        coil_y = client.read_coils(0,1)
        coil_r = client.read_coils(1,1)
        coil_b = client.read_coils(2,1)
        coil_m = client.read_coils(3,1)
        client.close()

        srv.yellow = coil_y.bits[0]
        srv.red = coil_r.bits[0]
        srv.blue = coil_b.bits[0]
        srv.missing = coil_m.bits[0]
    except:
        srv.yellow = 0
        srv.red = 0
        srv.blue = 0
        srv.missing = 1

    return srv


def mir_callback(req):
    srv = mir_checkResponse()
    try:
        client = ModbusTcpClient('192.168.1.202', 5020)
        x = client.read_holding_registers(0,1)
        y = client.read_holding_registers(1,1)
        marker_found = client.read_coils(4,1)
        client.close()

        srv.x = x.registers[0] - 128
        srv.y = y.registers[0] - 128
        srv.marker_found = marker_found.bits[0]
    except:
        srv.x = 0
        srv.y = 0
        srv.marker_found = 0

    return srv


def feeder_server():
    rospy.init_node('modbus_client')
    s = rospy.Service('feeder_check', feeder_srv, feeder_check)
    r = rospy.Service('mir_check', mir_check, mir_callback)
    print("Ready")
    rospy.spin()


if __name__ == "__main__":
    feeder_server()
