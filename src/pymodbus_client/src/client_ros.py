#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool
from pymodbus.client.sync import ModbusTcpClient
from pymodbus.register_read_message import ReadHoldingRegistersResponse


def talker():
    rospy.init_node('modbus_client', anonymous=True)
    pub_box = rospy.Publisher('check_box', Bool, queue_size=1)
    pub_feeder = rospy.Publisher('check_feeder', Bool, queue_size=1)
    client = ModbusTcpClient('localhost', 5020)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        coil1 = client.read_coils(0,1)
        coil2 = client.read_coils(1,1)

        pub_box.publish(coil1.bits[0])
        pub_feeder.publish(coil2.bits[0])
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
