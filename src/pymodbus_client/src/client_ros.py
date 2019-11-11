#!/usr/bin/env python3
import rospy
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse
from pymodbus_client.msg import feeder_msg


def talker():
    rospy.init_node('modbus_client', anonymous=True)
    pub_feeder = rospy.Publisher('feeder', feeder_msg, queue_size=1)
    client = ModbusTcpClient('localhost', 5020)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        coil1 = client.read_coils(0,1)
        coil2 = client.read_coils(1,1)
        coil3 = client.read_coils(2,1)
        msg = feeder_msg()
        msg.yellow =coil1.bits[0]
        msg.red =coil2.bits[0]
        msg.blue =coil3.bits[0]

        pub_feeder.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
