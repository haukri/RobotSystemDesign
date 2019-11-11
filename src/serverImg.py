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
from picamera.array import PiRGBArray
from picamera import PiCamera

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
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()


def check_image(image):
    S_min = 67
    V_min = 78

    S_max = 255
    V_max = 255

    result_y = False
    result_r = False
    result_b = False

    height, width = image.shape[:2]
    y=0
    w=300
    h=height

    #YELLOW
    x=700
    croppedImg = image[y:y+h, x:x+w]
    H_min = 17
    H_max = 30
    image_hsv = cv.cvtColor(croppedImg, cv.COLOR_BGR2HSV)
    lower = np.array([H_min,S_min,V_min])
    upper = np.array([H_max,S_max,V_max])
    image_seg = cv.inRange(image_hsv, lower, upper)
    count = cv.countNonZero(image_seg)
    if count > 23000:
        result_y = True

    #RED
    x=400
    croppedImg = image[y:y+h, x:x+w]
    H_min = 125
    H_max = 180
    image_hsv = cv.cvtColor(croppedImg, cv.COLOR_BGR2HSV)
    lower = np.array([H_min,S_min,V_min])
    upper = np.array([H_max,S_max,V_max])
    image_seg = cv.inRange(image_hsv, lower, upper)
    count = cv.countNonZero(image_seg)
    if count > 15000:
        result_r = True


    #BLUE
    x=100
    croppedImg = image[y:y+h, x:x+w]
    H_min = 82
    H_max = 118
    image_hsv = cv.cvtColor(croppedImg, cv.COLOR_BGR2HSV)
    lower = np.array([H_min,S_min,V_min])
    upper = np.array([H_max,S_max,V_max])
    image_seg = cv.inRange(image_hsv, lower, upper)
    count = cv.countNonZero(image_seg)
    if count > 7500:
        result_b = True


    return result_y, result_r, result_b


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
    address3 = 2
    values = context[slave_id].getValues(register, address3, count=1)

    rawCapture = PiRGBArray(camera)
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array

    result_y, result_r, result_b = check_image(image)
    log.debug("new values: " + str(result_y) + " " + str(result_r) + " " + str(result_b))
    log.debug("Time for update: " + str(time.time()-now))
    context[slave_id].setValues(register, address1, [result_y])
    context[slave_id].setValues(register, address2, [result_r])
    context[slave_id].setValues(register, address3, [result_b])


def run_updating_server():
    # ----------------------------------------------------------------------- #
    # initialize your data store
    # ----------------------------------------------------------------------- #

    store = ModbusSlaveContext(
        co=ModbusSequentialDataBlock(0, [0, 0, 0, 0]))
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
    time = 0.5  # 5 seconds delay
    loop = LoopingCall(f=updating_writer, a=context)
    loop.start(time, now=False) # initially delay by time
    StartTcpServer(context, identity=identity, address=("192.168.1.202", 5020))


if __name__ == "__main__":
    run_updating_server()
