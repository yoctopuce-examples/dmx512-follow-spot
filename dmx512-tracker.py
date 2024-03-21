#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import sys, math

from yocto_api import YAPI, YSensor, YRefParam
from yocto_serialport import YSerialPort

class DetectionLane:
    def __init__(self, sensor: YSensor, shift: int, maxDist: int):
        self.sensor = sensor
        self.shift = shift
        self.maxDist = maxDist
        self.event = None
        self.speed = 0
        # prepare to receive callbacks when something changes
        self.sensor.registerValueCallback(self.valueCallback)

    def valueCallback(self, fct: YSensor, value: str):
        newDist = int(value)
        if newDist < 50 or newDist > self.maxDist:
            self.event = None
            return
        if self.event is None:
            self.speed = 99999
        else:
            self.speed = newDist - self.event
        self.event = newDist

class PanTiltHead:
    def __init__(self, dmxPort: YSerialPort, height: int, distance: int):
        # configure RS485 interface for DMX512 standard
        self.dmxPort = dmxPort
        self.dmxPort.set_voltageLevel(YSerialPort.VOLTAGELEVEL_RS485)
        self.dmxPort.set_protocol("Frame:2ms")
        self.dmxPort.set_serialMode("250000,8N2")
        # pan tile head relative position
        self.height = height
        self.distance = distance
        # Default DMX512 frame to drive eurolite LED TMH-46 in 18CH mode
        self.dmxFrame = [ 0, 1, 1, 2, 2, 0, 255, 5, 6, 7, 8, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0  ]

    def hue2rgb(self, h: int):
        h = h & 255
        if h >= 170: return 1
        if h > 42:
            if h <= 127: return 255
            h = 170 - h
        return int(254 * 6 * h / 255) + 1

    def setFrame(self, dmxFrame: list[int], pan: int, tilt: int, hue: int|None):
        dmxFrame[1] = (pan >> 8) & 255
        dmxFrame[2] = pan & 255
        dmxFrame[3] = (tilt >> 8) & 255
        dmxFrame[4] = tilt & 255
        if hue is None:
            dmxFrame[8] = 0
            dmxFrame[9] = 0
            dmxFrame[10] = 0
            dmxFrame[11] = 0
            dmxFrame[12] = 0
            dmxFrame[13] = 0
        else:
            dmxFrame[8] = self.hue2rgb(hue + 85)  # R
            dmxFrame[9] = self.hue2rgb(hue)  # G
            dmxFrame[10] = self.hue2rgb(hue + 170)  # B
            dmxFrame[11] = 64
            dmxFrame[12] = 0
            dmxFrame[13] = 0

    def pointTo(self, lane: DetectionLane|None, hue: int|None):
        pan: int = 0
        tilt: int = 0
        if lane is not None:
            dist: int = self.distance - lane.event
            hypo = math.sqrt(lane.shift * lane.shift + dist * dist + self.height * self.height)
            pan = int(math.atan2(lane.shift, dist) * 21800 / math.pi + 21800)
            tilt = int(math.asin(self.height / hypo) * 65535 / math.pi)
        if (pan < 21800): pan += 2 * 21800
        self.setFrame(self.dmxFrame, pan, tilt, hue)
        self.dmxPort.sendBreak(1)
        self.dmxPort.writeArray(self.dmxFrame)


# Enable API log functions for debugging
def logfun(line):
    print('LOG : ' + line.rstrip())
YAPI.RegisterLogFunction(logfun)

# Setup the API to use devices on remove YoctoHub-Ethernet
errmsg = YRefParam()
if YAPI.RegisterHub("192.168.1.100", errmsg) != YAPI.SUCCESS:
    sys.exit("RegisterHub error: " + errmsg.value)

# locate Yoctopuce devices
dist1 = YSensor.FindSensor("dist1")
if not dist1.isOnline():
    sys.exit("RangeFinder #1 not found")
dist2 = YSensor.FindSensor("dist2")
if not dist2.isOnline():
    sys.exit("RangeFinder #2 not found")
dmxPortOut = YSerialPort.FindSerialPort("DMX-OUT.serialPort")
if not dmxPortOut.isOnline():
    sys.exit("No Yocto-RS485-V2 with logical name DMX-OUT found")

# lane 1 is 0.9m from head, length = 13m30
lane1 = DetectionLane(dist1, 900, 13300)
# lane 2 is 0.8m from head, length = 12m50
lane2 = DetectionLane(dist2, -800, 12500)
# pan tilt head is 2m70 above ground, 7m from sensors
panTiltHead = PanTiltHead(dmxPortOut, 2700, 7000)

print('************  DMX512 tracker - hit Ctrl-C to Stop ')

while True:
    # Wait for 20ms receiving rengefinder measurements
    YAPI.Sleep(20, errmsg)
    # Send a DMX message (~50 Hz)
    if lane2.event is None:
        if lane1.event is None:
            panTiltHead.pointTo(None, None)
        else:
            panTiltHead.pointTo(lane1, 270)
    else:
        if lane1.event is None or abs(lane1.speed) < abs(lane2.speed):
            panTiltHead.pointTo(lane2, 270)
        else:
            panTiltHead.pointTo(lane1, 270)
    x1 = -1 if lane1.event is None else lane1.event
    x2 = -1 if lane2.event is None else lane2.event
    print("\b\b\b\b\b\b\b\b\b\b\b\b\b\b{:5d} {:5d}".format(x1,x2), end='')
