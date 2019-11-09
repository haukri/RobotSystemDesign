#!/usr/bin/env python
"""
Pymodbus Server With Updating Thread
--------------------------------------------------------------------------

This is an example of having a background thread updating the
context while the server is operating. This can also be done with
a python thread::

    from threading import Thread

    thread = Thread(target=updating_writer, args=(context,))
    thread.start()
"""

import numpy as np
import cv2 as cv
import time
# --------------------------------------------------------------------------- #
# import the modbus libraries we need
# --------------------------------------------------------------------------- #
from pymodbus.server.asynchronous import StartTcpServer
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext
from pymodbus.transaction import ModbusRtuFramer, ModbusAsciiFramer

# --------------------------------------------------------------------------- #
# import the twisted libraries we need
# --------------------------------------------------------------------------- #
from twisted.internet.task import LoopingCall

# --------------------------------------------------------------------------- #
# configure the service logging
# --------------------------------------------------------------------------- #
import logging
logging.basicConfig()
log = logging.getLogger()
log.setLevel(logging.DEBUG)


def get_contours(image, H_min, H_max, S_min, S_max, V_min, V_max):
  image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
  #Detect the object based on HSV Range Values
  lower = np.array([H_min,S_min,V_min])
  upper = np.array([H_max,S_max,V_max])
  image_seg = cv.inRange(image_hsv, lower, upper)

  canny = cv.Canny( image_seg, 100, 200 )
  contours, hierarchy = cv.findContours(canny, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
  return contours


def check_order(blue, red, yellow, order):
    result = True
    # Count Blue
    if order[0] == len(blue):
        #Check if they are small
        for i in range(0, len(blue)):
            if cv.contourArea(blue[i])>60000:
                result = False
    else:
        result = False

    #Count Red
    if order[1] == len(red):
    #Check if they are normal
        for i in range(0, len(red)):
            if cv.contourArea(red[i])>60000:
                result = False
    else:
        result = False

    #Count Yellow
    if order[2] == len(yellow):
    #Check if they are normal
        for i in range(0, len(yellow)):
            if cv.contourArea(yellow[i])>60000:
                result = False
    else:
        result = False

    return result

# --------------------------------------------------------------------------- #
# define your callback process
# --------------------------------------------------------------------------- #

def updating_writer(a):
    """ A worker process that runs every so often and
    updates live values of the context. It should be noted
    that there is a race condition for the update.

    :param arguments: The input arguments to the call
    """
    log.debug("updating the context")
    now = time.time()
    context = a
    register = 1
    slave_id = 0x00
    address1 = 0
    address2 = 1
    values = context[slave_id].getValues(register, address1, count=1)
    if values[0] == 1:
        image = cv.imread("/home/lasse/Desktop/legoWrong.jpeg", cv.IMREAD_COLOR)
        print("Wrong image")
    else:
        image = cv.imread("/home/lasse/Desktop/lego.png", cv.IMREAD_COLOR)
        print("Right Image")

    order = [2, 1, 1]

    contours_blue = get_contours(image, 52, 118, 72, 255, 0, 255)
    contours_red = get_contours(image, 0, 15, 72, 255, 0, 255)
    contours_yellow = get_contours(image, 16, 36, 72, 255, 0, 255)

    result = check_order(contours_blue, contours_red, contours_yellow, order)
    res = [result]
    log.debug("new values: " + str(res))
    log.debug("Time for update: " + str(time.time()-now))
    context[slave_id].setValues(register, address1, res)
    context[slave_id].setValues(register, address2, res)


def run_updating_server():
    # ----------------------------------------------------------------------- #
    # initialize your data store
    # ----------------------------------------------------------------------- #

    store = ModbusSlaveContext(
        co=ModbusSequentialDataBlock(0, [0, 0, 0]))
    context = ModbusServerContext(slaves=store, single=True)

    # ----------------------------------------------------------------------- #
    # initialize the server information
    # ----------------------------------------------------------------------- #
    identity = ModbusDeviceIdentification()
    identity.VendorName = 'pymodbus'
    identity.ProductCode = 'PM'
    identity.VendorUrl = 'http://github.com/bashwork/pymodbus/'
    identity.ProductName = 'pymodbus Server'
    identity.ModelName = 'pymodbus Server'
    identity.MajorMinorRevision = '2.3.0'

    # ----------------------------------------------------------------------- #
    # run the server you want
    # ----------------------------------------------------------------------- #
    time = 5  # 5 seconds delay
    loop = LoopingCall(f=updating_writer, a=context)
    loop.start(time, now=False) # initially delay by time
    StartTcpServer(context, identity=identity, address=("localhost", 5020))


if __name__ == "__main__":
    run_updating_server()
