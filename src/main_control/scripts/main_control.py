#!/usr/bin/env python

import rospy
from robot_msgs.msg import RobotCmd, RobotStatus
from order_msgs.srv import NewOrder, CompleteOrder
from packml_msgs.msg import Status
import threading
import json
from Queue import Queue

robotCommandPub = 0
packmlState = 2
stateChangeQueue = Queue()
robotReady = True
newOrder = 0
completeOrder = 0
currentOrder = 0
substate = 0
binNumber = 0
currentRobotCmd = 0
lastState = 0

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
    global stateChangeQueue, lastState
    if data.state.val != lastState:
        stateChangeQueue.put(data.state.val)
        lastState = data.state.val


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


def saveOrder():
    global currentOrder, binNumber
    with open('currentOrder.json', 'w+') as f:
        f.write(json.dumps([currentOrder.blue_amount, currentOrder.red_amount, currentOrder.yellow_amount, currentOrder.order_number]))
    with open('binNumber.json', 'w+') as f:
        f.write(json.dumps([binNumber]))


def loadOrder():
    global currentOrder, binNumber
    content = ""
    with open('currentOrder.json', 'r') as f:
        content = f.read()
    if(content != ""):
        print("Loading saved order")
        order = json.loads(content)
        currentOrder = type('', (), {})()
        currentOrder.blue_amount = order[0]
        currentOrder.red_amount = order[1]
        currentOrder.yellow_amount = order[2]
        currentOrder.order_number = order[3]
        return True
    with open('binNumber.json', 'r') as f:
        content = f.read()
    if(content != ""):
        data = json.loads(content)
        binNumber = data[0]
    return False


def deleteOrder():
    with open('currentOrder.json', 'w+') as f:
        f.write('')
    with open('binNumber.json', 'w+') as f:
        f.write(json.dumps([binNumber]))


def publisher():
    global packmlState, currentOrder, robotReady, newOrder, completeOrder, binNumber, substate, currentRobotCmd, stateChangeQueue
    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        if(not stateChangeQueue.empty()):
            packmlState = stateChangeQueue.get()
        if packmlState == EXECUTE:
            if substate == 0:                # Get new order
                print("0: New order")
                if(not loadOrder()):
                    print("No saved order found")
                    currentOrder = newOrder()
                    saveOrder()
                substate = 10
            elif substate == 5:
                print("5: New order")
                currentOrder = newOrder()
                substate = 10
            elif substate == 10:             # Command robot for next brick
                if not orderDone():
                    currentRobotCmd = RobotCmd()
                    currentRobotCmd.command = getNextBrick()
                    currentRobotCmd.binNumber = binNumber
                    robotCommandPub.publish(currentRobotCmd)
                    print("10: Packing brick")
                    robotReady = False
                    substate = 20
                else:
                    print("10: Order done")
                    substate = 5
                    completeOrder(order_number=currentOrder.order_number)
                    if(binNumber == 4):     # If all bins have been packed, call MiR robot for pickup
                        substate = 30
                    deleteOrder()
            elif substate == 20:            # Wait for robot to complete move
                if robotReady:
                    print("20: Packing brick done")
                    substate = 10
            elif substate == 30:            # Call MiR robot for pickup
                print("30: Call MiR robot for pickup")
                substate = 40
            elif substate == 40:            
                print("40: Wait for MiR to arrive")
                substate = 50
            elif substate == 50:            
                print("50: Transfer boxes over to MiR")
                substate = 60
            elif substate == 60:            
                print("60: Wait for transfer done")
                substate = 5
        elif packmlState == STARTING:
            if substate < 30:
                substate = 0
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