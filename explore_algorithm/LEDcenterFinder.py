# -*- coding: utf-8 -*-
import numpy as np

# --- Local ---
from CentralRecognition import CentralRecognition
from calculation import Calc
from video import Video

class LEDCenterFinder() :

    def __init__(self) :
        self.video_cap = Video()
        self.central_recognizer = CentralRecognition()
        self.calculator = Calc()

    def __video_capture(self) :
        self.video_cap.capture(self.VIDEO_NAME, self.shutter_speed)

    # Detects real marker
    def __marker_detection(self) :
        self.__marker_detection_img = self.central_recognizer.main(self.VIDEO_NAME, self.MARKER_FREQ)

    # Calculate the distance and angle to the marker
    def __calc_distance_and_phi(self) :
        self.__distance, self.__azimuth_angle,self.__x,self.__y= self.calculator.main(self.__marker_detection_img, self.MARKER_FREQ, self.virtualmarkerxcoord, self.virtualmarkerycoord, self.betweenmarker, self.GYRO_ANGLES)

    # Return distance[m] and angle(phi)[deg]
    def getRTheta(self, VIDEO_NAME : int, MARKER_FREQ : str, GYRO_ANGLES, virtualmarkerxcoord = 0, virtualmarkerycoord = 0, shutter_speed=100, betweenmarker=0.8) :
        self.VIDEO_NAME = str(VIDEO_NAME)
        MARKER_FREQ = MARKER_FREQ.split(sep='_')
        self.MARKER_FREQ = [int(_) for _ in MARKER_FREQ]
        self.GYRO_ANGLES = GYRO_ANGLES
        self.virtualmarkerxcoord = virtualmarkerxcoord
        self.virtualmarkerycoord = virtualmarkerycoord
        self.shutter_speed = shutter_speed
        self.betweenmarker = betweenmarker

        self.__video_capture()
        self.__marker_detection()
        self.__calc_distance_and_phi()

        self.__distance = list((self.__distance.reshape(-1)))
        self.__azimuth_angle = list(np.rad2deg(self.__azimuth_angle).reshape(-1))
        self.__x = list((self.__x.reshape(-1)))
        self.__y = list((self.__y.reshape(-1)))
        return self.__distance, self.__azimuth_angle,self.__x,self.__y

if __name__ == '__main__' :
    x = LEDCenterFinder()
    VIDEO_NAME = input('VIDEO NAME : ')
    #VIDEO_NAME = '300'
    MARKER_FREQ = '7'
    GYRO_ANGLES = [0, 10]
    result = x.getRTheta(VIDEO_NAME, MARKER_FREQ, GYRO_ANGLES)
    print(result)