#!/usr/bin/env python3
import rospy
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse
from pymodbus_client.srv import feeder_srv, feeder_srvResponse

def feeder_check(req):
    result = False
    client = ModbusTcpClient('192.168.1.202', 5020)

#    if req.colour == "YELLOW":
#        coil = client.read_coils(0,1)
#        result = coil.bits[0]
#    elif req.colour == "RED":
#        coil = client.read_coils(1,1)
#        result = coil.bits[0]
#    elif req.colour == "BLUE":
#        coil = client.read_coils(2,1)
#        result = coil.bits[0]

    coil_y = client.read_coils(0,1)
    coil_r = client.read_coils(1,1)
    coil_b = client.read_coils(2,1)

    srv = feeder_srvResponse()
    srv.yellow = coil_y.bits[0]
    srv.red = coil_r.bits[0]
    srv.blue = coil_b.bits[0]
    client.close()
    return srv

def feeder_server():
    rospy.init_node('modbus_client')
    s = rospy.Service('feeder_check', feeder_srv, feeder_check)
    print("Ready")
    rospy.spin()

if __name__ == "__main__":
    feeder_server()
