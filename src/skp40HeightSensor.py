#! /usr/bin/env python

import threading
from os import system, path, readlink
from struct import pack, unpack
from sys import exit
from time import time, sleep
from collections import deque
from math import degrees, radians

import rospy
import serial
from geometry_msgs.msg import Vector3Stamped

HZ = 50

DOWN_FRAME_HEAD_1 = b'\xEA'
DOWN_FRAME_HEAD_2 = b'\x9B'
DOWN_FRAME_HEAD = DOWN_FRAME_HEAD_1 + DOWN_FRAME_HEAD_2

DOWN_FRAME_TAIL = b'\xAA'
FRAME_LEN = 78

bAny, bS8, bU8, bS16, bU16, bS32, bU32, bF = 'x', 'b', 'B', 'h', 'H', 'i', 'I', 'f'
DOWN_PROTO = '<' + bAny * 40 + bS16 * 3 + bAny + bS16 * 6 + bAny * 19

WAITING_DOWN_FRAME_HEAD_1 = 1
WAITING_DOWN_FRAME_HEAD_2 = 2
READING_DATA = 3

from glob import glob

PORT = glob('/dev/ttyUSB0')[0]
# PORT = path.join('/dev', readlink(glob('/dev/serial/by-path/*usb-0:2.2:1.0*')[0]).split('/')[-1])

from signal import signal, SIGINT


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    exit(0)


signal(SIGINT, signal_handler)


def timer(tol=1):
    def decorator(func):
        def wrapper(*args, **kwargs):
            startTime = time()
            result = func(*args, **kwargs)
            endTime = time()
            # print(f'Time elap: {endTime - startTime:.2f}')
            if endTime - startTime < tol:
                sleep(tol - endTime + startTime)
            # print(f'Ended')
            return result

        return wrapper

    return decorator


class HEIGHTSENSORCOMM:
    def __init__(self):
        self.init = False
        self.state = WAITING_DOWN_FRAME_HEAD_1

        self.height = -1.0
        self.heightDeque = deque(maxlen=50)
        self.heightAvg = -1.0

        self.downSer = serial.Serial(
            port=PORT,
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=0.5
        )
        self.dataBuf = bytearray()

        # rospy.init_node('height_sensor_comm', anonymous=True)
        # rospy.Rate(10)
        # self.heightPub = rospy.Publisher('/skp40/height', Vector3Stamped, queue_size=1)

    def getTime_now(self):
        return time()

    def readData(self):
        while True:
            data = self.downSer.read(1)
            if data:
                if self.state == WAITING_DOWN_FRAME_HEAD_1:
                    if data == DOWN_FRAME_HEAD_1:
                        self.state = WAITING_DOWN_FRAME_HEAD_2
                elif self.state == WAITING_DOWN_FRAME_HEAD_2:
                    if data == DOWN_FRAME_HEAD_2:
                        self.state = READING_DATA
                        dataBuf = bytearray()
                elif self.state == READING_DATA:
                    # print(f'reading data buffer len {len(dataBuf)}')
                    dataBuf.append(data[0])
                    if len(dataBuf) == FRAME_LEN:
                        #print('down: ', DOWN_FRAME_HEAD.hex(), dataBuf[:-1].hex(), dataBuf[-1:].hex())
                        downData = unpack(DOWN_PROTO, dataBuf)

                        yaw, pitch, roll, accX, accY, accZ, gyroX, gyroY, gyroZ = downData

                        yaw /= 1000.0
                        pitch /= 1000.0
                        roll /= 1000.0
                        accX /= 100.0
                        accY /= 100.0
                        accZ /= 100.0
                        gyroX /= 100.0
                        gyroY /= 100.0
                        gyroZ /= 100.0

                        print('Yaw: ', degrees(yaw))
                        print('Pitch: ', degrees(pitch))
                        print('Roll: ', degrees(roll))
                        print('acc: ', accX, accY, accZ)
                        print('gyro: ', gyroX, gyroY, gyroZ)

                        self.state = WAITING_DOWN_FRAME_HEAD_1

            if self.state == WAITING_DOWN_FRAME_HEAD_1:
                dataBuf = bytearray()

    def startRead(self):
        tRead = threading.Thread(target=self.readData)
        tRead.start()

    def rosPub(self):
        msg = Vector3Stamped()
        msg.vector.x = self.height
        msg.vector.y = self.heightAvg
        self.heightPub.publish(msg)

    @timer(tol=1 / HZ)
    def spinOnce(self):
        self.rosPub()

    def spin(self):
        self.startRead()
        while not rospy.is_shutdown():
            self.spinOnce()


if __name__ == '__main__':
    print('PORT is', PORT)
    hsc = HEIGHTSENSORCOMM()
    hsc.spin()
