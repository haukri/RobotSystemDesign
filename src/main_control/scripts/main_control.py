#!/usr/bin/env python
import rospy
from robot_msgs.msg import RobotCmd, RobotStatus
from order_msgs.srv import NewOrder, CompleteOrder
from packml_msgs.msg import Status
import threading

robotCommandPub = 0
packmlState = 2
robotReady = True
newOrder = 0
completeOrder = 0
currentOrder = 0
substate = 0
binNumber = 0
currentRobotCmd = 0

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


def packml_callback(data):
    global packmlState
    packmlState = data.state.val


def robot_callback(data):
    global robotReady
    robotReady = data.ready


def getNextBrick():
    if currentOrder.blue_amount > 0:
        currentOrder.blue_amount = currentOrder.blue_amount - 1
        return "pick-blue"
    elif currentOrder.red_amount > 0:
        currentOrder.red_amount = currentOrder.red_amount - 1
        return "pick-red"
    elif currentOrder.yellow_amount > 0:
        currentOrder.yellow_amount = currentOrder.yellow_amount - 1
        return "pick-yellow"


def orderDone():
    return currentOrder.blue_amount == 0 and currentOrder.red_amount == 0 and currentOrder.yellow_amount == 0


def publisher():
    global packmlState, currentOrder, robotReady, newOrder, completeOrder, binNumber, substate, currentRobotCmd
    r = rospy.Rate(0.1)
    while not rospy.is_shutdown():
        if packmlState == EXECUTE:
            if substate == 0:                # Get new order
                print("new order")
                currentOrder = newOrder()
                substate = 10
            elif substate == 10:             # Command robot for next brick
                if not orderDone():
                    currentRobotCmd = RobotCmd()
                    currentRobotCmd.command = getNextBrick()
                    currentRobotCmd.binNumber = binNumber
                    robotCommandPub.publish(currentRobotCmd)
                    print("packing brick")
                    robotReady = False
                    substate = 20
                else:
                    print("order done")
                    completeOrder(order_number=currentOrder.order_number)
                    substate = 0
            elif substate == 20:             # Wait for robot to complete move
                if robotReady:
                    print("packing brick done")
                    substate = 10
            elif substate == 300:            # Robot was stopped
                pass
        if packmlState == STOPPED:
            pass
            
        r.sleep()


def listener():
    global robotCommandPub, newOrder, completeOrder
    robotCommandPub = rospy.Publisher('robot_command_new', RobotCmd, queue_size=10)

    rospy.init_node('main_control', anonymous=True)
    
    rospy.Subscriber("packml_node/packml/status", Status, packml_callback)
    rospy.Subscriber("robot_command_status", RobotStatus, robot_callback)
    
    rospy.wait_for_service('new_order')
    rospy.wait_for_service('complete_order')

    newOrder = rospy.ServiceProxy('new_order', NewOrder)
    completeOrder = rospy.ServiceProxy('complete_order', CompleteOrder)

    thread = threading.Thread(target=publisher)
    thread.start()

    rospy.spin()


if __name__ == '__main__':
    listener()