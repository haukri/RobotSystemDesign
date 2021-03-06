#!/usr/bin/env python

import rospy
from robot_msgs.msg import RobotCmd, RobotStatus
from order_msgs.srv import NewOrder, CompleteOrder
from std_msgs.msg import String
from pymodbus_client.srv import feeder_srv
from packml_msgs.msg import Status
from packml_msgs.srv import Transition
import threading
import json
from Queue import Queue
from notification import sendPushNotification

robotCommandPub = 0
mainStatusPub = 0
oeeCommandsPub = 0
feederEmptyPub = 0
packmlState = 2
stateChangeQueue = Queue()
robotReady = True
newOrder = 0
completeOrder = 0
feederCheck = 0
currentOrder = 0
substate = 0
binNumber = 1
currentRobotCmd = 0
lastState = 0
bricksValid = 0
hasDiscardedBricks = False
bricksVerified = type('', (), {})()
bricksVerified.blue = False
bricksVerified.red = False
bricksVerified.yellow = False
message = ""
lastMessage = ""
goodOrder = True
packmlTransitionCommand = 0

feederStatus = type('', (), {})()
feederStatus.max_yellow_amount = 13
feederStatus.max_red_amount = 19
feederStatus.max_blue_amount = 37
feederStatus.yellow_amount = 13
feederStatus.red_amount = 19
feederStatus.blue_amount = 37
feederStatus.empty = False

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


def feeder_callback(data):
    global feederStatus
    feederStatus.empty = False


def getNextBrick():
    global feederStatus
    if currentOrder.blue_amount > 0:
        currentOrder.blue_amount = currentOrder.blue_amount - 1
        feederStatus.blue_amount = feederStatus.blue_amount - 1
        return "pick-blue"
    elif currentOrder.red_amount > 0:
        currentOrder.red_amount = currentOrder.red_amount - 1
        feederStatus.red_amount = feederStatus.red_amount - 1
        return "pick-red"
    elif currentOrder.yellow_amount > 0:
        currentOrder.yellow_amount = currentOrder.yellow_amount - 1
        feederStatus.yellow_amount = feederStatus.yellow_amount - 1
        return "pick-yellow"
    saveOrder()


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
    global packmlState, currentOrder, message, goodOrder, lastPackmlState, feederStatus, hasDiscardedBricks, bricksValid, feederCheck, robotReady, newOrder, completeOrder, binNumber, substate, currentRobotCmd, stateChangeQueue
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if(not stateChangeQueue.empty()):
            packmlState = stateChangeQueue.get()
            substate = 0
            print('PackML state changed!')

        if packmlState == EXECUTE:
            if substate == 0:                # Get new order
                message = "0: load order"
                if(not loadOrder()):
                    message = "No saved order found"
                    substate = 5
                else:
                    substate = 10
            elif substate == 5:
                message = "5: New order"
                currentOrder = newOrder()
                goodOrder = True
                saveOrder()
                substate = 10
            elif substate == 10:             # Command robot for next brick
                if not orderDone():
                    if feederIsEmpty():
                        suspendMachine()
                        msg = String()
                        feederEmptyPub.publish(msg)
                        feederStatus.empty = True
                        sendPushNotification('Feeder is empty! please refill the feeder')
                        substate = 13
                    else:
                        currentRobotCmd = RobotCmd()
                        currentRobotCmd.command = 'verify-bricks'
                        currentRobotCmd.binNumber = 1
                        robotCommandPub.publish(currentRobotCmd)
                        message = "10: Verifying bricks"
                        robotReady = False
                        substate = 11
                else:
                    message = "10: Order done"
                    substate = 5
                    completeOrder(currentOrder.order_number)
                    if(binNumber == 4):     # If all bins have been packed, call MiR robot for pickup
                        substate = 30
                        binNumber = 1
                    else:
                        binNumber = binNumber + 1
                    deleteOrder()
                    if goodOrder:
                        publishGoodOrder()
                    else:
                        publishBadOrder()
            elif substate == 11:            # Discard all bricks that are not valid
                if robotReady:
                    # Call verify brick service
                    # rospy.sleep(0.5)
                    bricksValid = feederCheck()
                    hasDiscardedBricks = False
                    substate = 12
            elif substate == 12:
                if not bricksValid.red:
                    bricksValid.red = True
                    currentRobotCmd = RobotCmd()
                    currentRobotCmd.command = 'discard-red'
                    robotCommandPub.publish(currentRobotCmd)
                    robotReady = False
                    feederStatus.red_amount = feederStatus.red_amount - 1
                    substate = 18
                elif not bricksValid.blue:
                    bricksValid.blue = True
                    currentRobotCmd = RobotCmd()
                    currentRobotCmd.command = 'discard-blue'
                    robotCommandPub.publish(currentRobotCmd)
                    robotReady = False
                    feederStatus.blue_amount = feederStatus.blue_amount - 1
                    substate = 18
                elif not bricksValid.yellow:
                    bricksValid.yellow = True
                    currentRobotCmd = RobotCmd()
                    currentRobotCmd.command = 'discard-yellow'
                    robotCommandPub.publish(currentRobotCmd)
                    robotReady = False
                    feederStatus.yellow_amount = feederStatus.yellow_amount - 1
                    substate = 18
                else:
                    if hasDiscardedBricks:
                        substate = 10
                    else:
                        substate = 19
            elif substate == 13:
                message = "13: Feeder is empty, waiting for state suspended"
            elif substate == 18:            # Wait for robot to finish executing discard move
                if robotReady:
                    goodOrder = False
                    hasDiscardedBricks = True
                    substate = 12     
            elif substate == 19:            # Get next brick and make message
                message = "19: Packing brick"
                currentRobotCmd = RobotCmd()
                currentRobotCmd.command = getNextBrick()
                currentRobotCmd.binNumber = binNumber
                robotCommandPub.publish(currentRobotCmd)
                robotReady = False
                substate = 20
            elif substate == 20:            # Wait for robot to complete move
                if robotReady:
                    message = "20: Packing brick done"
                    substate = 10
            elif substate == 25:            # Moving boxes to waiting zone
                message = "25: Moving boxes to waiting zone"
                currentRobotCmd = RobotCmd()
                currentRobotCmd.command = 'move-boxes-waiting'
                robotCommandPub.publish(currentRobotCmd)
                robotReady = False
                substate = 26
            elif substate == 26:            # Wait for robot finishing moving boxes to waiting zone
                if robotReady:
                    substate = 30
            elif substate == 30:            # Call MiR robot for pickup
                message = "30: Call MiR robot for pickup"
                substate = 40
            elif substate == 40:            
                message = "40: Wait for MiR to arrive"
                substate = 50
            elif substate == 50:            
                message = "50: Find box position offset on MiR"
                currentRobotCmd = RobotCmd()
                currentRobotCmd.command = 'camera-over-mir'
                robotCommandPub.publish(currentRobotCmd)
                robotReady = False
                substate = 60
            elif substate == 60:            
                if robotReady:
                    substate = 70
            elif substate == 70:
                message = "70: Request position offsets from camera"
                substate = 80
            elif substate == 80:
                message = "80: Move empty boxes from MiR to table"
                currentRobotCmd = RobotCmd()
                currentRobotCmd.command = 'move-boxes-from-mir'
                currentRobotCmd.x_offset = 0.0 # TODO add offsets from camera
                currentRobotCmd.y_offset = 0.0 # TODO add offsets from camera
                robotCommandPub.publish(currentRobotCmd)
                robotReady = False
                substate = 95
            elif substate == 95:
                if robotReady:
                    substate = 100
            elif substate == 100:
                message = "100: Move boxes from waiting zone to MiR"
                currentRobotCmd = RobotCmd()
                currentRobotCmd.command = 'move-boxes-to-mir'
                currentRobotCmd.x_offset = 0.0 # TODO add offsets from camera
                currentRobotCmd.y_offset = 0.0 # TODO add offsets from camera
                robotCommandPub.publish(currentRobotCmd)
                robotReady = False
                substate = 105
            elif substate == 105:
                if robotReady:
                    substate = 110
            elif substate == 110:
                message = "100: Request MiR to go away"
                substate = 5

        elif packmlState == SUSPENDED:
            if substate == 0:
                message = "0: Waiting for feeder to be full"
                if not feederStatus.empty:
                    feederStatus.blue_amount = feederStatus.max_blue_amount
                    feederStatus.red_amount = feederStatus.max_red_amount
                    feederStatus.yellow_amount = feederStatus.max_yellow_amount
                    substate = 10
                    unsuspendMachine()
            elif substate == 10:
                message = "10: Waiting for transition to state execute"

        publishStatus()
        r.sleep()


def suspendMachine():
    packmlTransitionCommand(100)


def unsuspendMachine():
    packmlTransitionCommand(101)


def feederIsEmpty():
    return feederStatus.blue_amount <= 5 or feederStatus.red_amount <= 4 or feederStatus.yellow_amount <= 3


def publishStatus():
    global message, lastMessage
    if message != lastMessage:
        print(message)
        msg = String()
        msg.data = message
        mainStatusPub.publish(msg)
    lastMessage = message


def publishBadOrder():
    msg = String()
    msg.data = 'bad-brick'
    oeeCommandsPub.publish(msg)


def publishGoodOrder():
    msg = String()
    msg.data = 'good-brick'
    oeeCommandsPub.publish(msg)


def listener():
    global robotCommandPub, mainStatusPub, newOrder, completeOrder, feederCheck, oeeCommandsPub

    robotCommandPub = rospy.Publisher('robot_command_new', RobotCmd, queue_size=10)
    mainStatusPub = rospy.Publisher('main_control_status', String, queue_size=10)
    oeeCommandsPub = rospy.Publisher("packml_node/packml/oee_commands", String, queue_size=10)
    feederEmptyPub = rospy.Publisher("feeder_empty", String, queue_size=10)

    rospy.init_node('main_control', anonymous=True)
    
    rospy.Subscriber("packml_node/packml/status", Status, packml_callback)
    rospy.Subscriber("robot_command_status", RobotStatus, robot_callback)
    rospy.Subscriber("feeder_full", RobotStatus, feeder_callback)
    
    rospy.wait_for_service('new_order')
    rospy.wait_for_service('complete_order')
    rospy.wait_for_service('feeder_check')
    rospy.wait_for_service('packml_node/packml/transition')

    newOrder = rospy.ServiceProxy('new_order', NewOrder)
    completeOrder = rospy.ServiceProxy('complete_order', CompleteOrder)
    feederCheck = rospy.ServiceProxy('feeder_check', feeder_srv)
    packmlTransitionCommand = rospy.ServiceProxy('packml_node/packml/transition', Transition)

    thread = threading.Thread(target=publisher)
    thread.start()

    rospy.spin()


if __name__ == '__main__':
    listener()