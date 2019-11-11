#!/usr/bin/env python
import rospy
from robot_msgs.msg import RobotIO
from packml_msgs.msg import Status
import threading

pub = 0
state = 2
red = False
yellow = False
green = False

STOPPED = 2
STARTING = 3
IDLE = 4
SUSPENDED = 5
EXECUTE = 6
STOPPING = 7
ABORTING = 8
ABORTED = 9
HOLDING = 10
HELD = 11

RESETTING = 100
SUSPENDING = 101
UNSUSPENDING = 102
CLEARING = 103
UNHOLDING = 104
COMPLETING = 105
COMPLETE = 106

packml = {}
packml[ABORTING] = ('F', '', '')
packml[ABORTED] = ('F', '', '')
packml[CLEARING] = ('F', '', '')
packml[STOPPING] = ('S', '', '')
packml[STOPPED] = ('S', '', '')
packml[RESETTING] = ('', 'F', '')
packml[IDLE] = ('', '', 'F')
packml[STARTING] = ('', '', 'S')
packml[EXECUTE] = ('', '', 'S')
packml[HOLDING] = ('', 'F', 'F')
packml[HELD] = ('', 'F', 'F')
packml[UNHOLDING] = ('', '', 'S')
packml[SUSPENDING] = ('', 'S', '')
packml[SUSPENDED] = ('', 'S', '')
packml[UNSUSPENDING] = ('', '', 'S')
packml[COMPLETING] = ('', 'S', 'S')
packml[COMPLETE] = ('', 'S', 'S')


def callback(data):
    global state, red, yellow, green
    state = data.state.val
    red = False
    yellow = False
    green = False

def publisher():
    global level, red, yellow, green, packml
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        msg = RobotIO()
        lights = packml[state]
        if lights[0] == 'S':
            red = True
        elif lights[0] == 'F':
            red = not red
        else:
            red = False
        msg.output = 0
        msg.level = red
        pub.publish(msg)

        if lights[1] == 'S':
            yellow = True
        elif lights[1] == 'F':
            yellow = not yellow
        else:
            yellow = False
        msg.output = 1
        msg.level = yellow
        pub.publish(msg)

        if lights[2] == 'S':
            green = True
        elif lights[2] == 'F':
            green = not green
        else:
            green = False
        msg.output = 2
        msg.level = green
        pub.publish(msg)

        r.sleep()
    
def listener():
    global pub
    pub = rospy.Publisher('robot_io', RobotIO, queue_size=10)

    rospy.init_node('light_tower', anonymous=True)
    
    rospy.Subscriber("packml_node/packml/status", Status, callback)
    
    thread = threading.Thread(target=publisher)
    thread.start()

    rospy.spin()

if __name__ == '__main__':
    listener()