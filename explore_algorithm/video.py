# -*- coding:utf-8 -*-
import picamera
import os
from time import sleep
import sys
import cv2
import picamera.array
import numpy as np

class Video() :
    __VIDEO_DIR = 'video/'
    __VIDEO_EXT = '.h264'
    __IMG_DIR = ''
    __IMG_EXT = '.jpg'

    def __init__(self) :
        self.cap = picamera.PiCamera()
        self.cap.hflip = True
        self.cap.vflip = True
        self.cap.sensor_mode = 4
        self.cap.framerate = 30
        self.cap.resolution = (1024, 768)
        if not os.path.exists(self.__VIDEO_DIR) :
            os.makedirs(self.__VIDEO_DIR)

        self.cap.shutter_speed = 10000
        sleep(2)

    def capture(self, VIDEO_NAME : str, shutter_speed) :
        if not self.cap.shutter_speed == shutter_speed :
            self.cap.shutter_speed = shutter_speed
            sleep(2)
        self.cap.start_recording(os.path.join(self.__VIDEO_DIR, VIDEO_NAME + self.__VIDEO_EXT))
        self.cap.wait_recording(1.25)
        self.cap.stop_recording()

    def frame(self, IMG_NAME : str, SHUTTER_SPEED : int, EXPLORATION_TAG:str) :
        if not self.cap.shutter_speed == SHUTTER_SPEED :
            self.cap.shutter_speed = SHUTTER_SPEED
            sleep(2)
        self.__IMG_DIR = EXPLORATION_TAG + "-Ceil"
        os.makedirs("./" + self.__IMG_DIR, exist_ok=True)
        rawCapture = picamera.array.PiRGBArray(self.cap, size=(1024, 768))
        self.cap.capture(rawCapture, 'bgr')
        frame = rawCapture.array
        cv2.imwrite(os.path.join(self.__IMG_DIR, IMG_NAME + self.__IMG_EXT), frame)

    def __del__(self) :
        del self.cap

if __name__ == '__main__' :
    cap = Video()
    cap.capture('1')
    cap.frame('1')
