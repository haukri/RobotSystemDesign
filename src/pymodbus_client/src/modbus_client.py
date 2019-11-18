#!/usr/bin/env python
import rospy
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse
from pymodbus_client.srv import mir_check, mir_checkResponse, feeder_srv, feeder_srvResponse


def feeder_check(req):
    client = ModbusTcpClient('192.168.1.202', 5020)
    coil_y = client.read_coils(0,1)
    coil_r = client.read_coils(1,1)
    coil_b = client.read_coils(2,1)
    coil_m = client.read_coils(3,1)
    client.close()

    srv = feeder_srvResponse()
    srv.yellow = coil_y.bits[0]
    srv.red = coil_r.bits[0]
    srv.blue = coil_b.bits[0]
    srv.missing = coil_m.bits[0]
    return srv


def mir_callback(req):
    client = ModbusTcpClient('192.168.1.202', 5020)

    x = client.read_holding_registers(0,1)
    y = client.read_holding_registers(1,1)
    client.close()

    srv = mir_checkResponse()
    srv.x = x.registers[0]
    srv.y = y.registers[0]
    return srv


def feeder_server():
    rospy.init_node('modbus_client')
    s = rospy.Service('feeder_check', feeder_srv, feeder_check)
    r = rospy.Service('mir_check', mir_check, mir_callback)
    print("Ready")
    rospy.spin()


if __name__ == "__main__":
    feeder_server()
