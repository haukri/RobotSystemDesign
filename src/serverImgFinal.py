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
import cv2.aruco as aruco

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
    V_min = 40

    S_max = 255
    V_max = 255

    pos_x = [881, 542, 211]
    pos_y = [202, 216, 211]

    result_y = False
    result_r = False
    result_b = False
    missing = False

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
    if count > 19500:
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
    if count > 13000:
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
    if count > 6500:
        result_b = True

    H_min = 0
    S_min = 0
    V_min = 0
    H_max = 180
    S_max = 130
    V_max = 255
    image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    lower = np.array([H_min,S_min,V_min])
    upper = np.array([H_max,S_max,V_max])
    image_seg = cv.inRange(image_hsv, lower, upper)
    for i in range(3):
        x = pos_x[i]
        y = pos_y[i]
        if image_seg[y, x] == 255:
            missing = True



    return result_y, result_r, result_b, missing

def detect_marker(image):
    pixels = 665-237
    mmPpixel = 1500/pixels #0.1 mm pr pixel
    ground_truth = [628, 384]

    ARUCO_PARAMETERS = aruco.DetectorParameters_create()
    dictionary = aruco.Dictionary_get(aruco.DICT_5X5_50)

    corners, ids, rejectedImgPoints = aruco.detectMarkers(image, dictionary, parameters=ARUCO_PARAMETERS)
    error = (ground_truth - corners[0][0][0])*mmPpixel
    #print("error", error)
    error_x = error[0]
    error_y = error[1]

    return int(error_x), int(error_y)


# --------------------------------------------------------------------------- #
# define your callback process
# --------------------------------------------------------------------------- #

def updating_writer(a):
    """ A worker process that runs every so often and
    updates live values of the context. It should be noted
    that there is a race condition for the update.

    :param arguments: The input arguments to the call
    """
    #log.debug("updating the context")
    now = time.time()
    context = a
    register_coil = 1
    register_holding = 3
    slave_id = 0x00
    address_yellow = 0
    address_red = 1
    address_blue = 2
    address_missing = 3
    address_x = 0
    address_y = 1

    rawCapture = PiRGBArray(camera)
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array

    result_y, result_r, result_b, missing = check_image(image)
    error_x, error_y = detect_marker(image)

    #log.debug("new values: " + str(result_y) + " " + str(result_r) + " " + str(result_b))
    #log.debug("Time for update: " + str(time.time()-now))
    context[slave_id].setValues(register_coil, address_yellow, [result_y])
    context[slave_id].setValues(register_coil, address_red, [result_r])
    context[slave_id].setValues(register_coil, address_blue, [result_b])
    context[slave_id].setValues(register_coil, address_missing, [missing])
    context[slave_id].setValues(register_holding, address_x, [error_x])
    context[slave_id].setValues(register_holding, address_y, [error_y])


def run_updating_server():
    # ----------------------------------------------------------------------- #
    # initialize your data store
    # ----------------------------------------------------------------------- #

    store = ModbusSlaveContext(
            co=ModbusSequentialDataBlock(0, [0]*10),
            hr=ModbusSequentialDataBlock(0, [5]*10))
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
    time = 0.1  # 0.2 seconds delay
    loop = LoopingCall(f=updating_writer, a=context)
    loop.start(time, now=False) # initially delay by time
    StartTcpServer(context, identity=identity, address=("192.168.1.202", 5020))


if __name__ == "__main__":
    run_updating_server()
