# -*- coding:utf-8 -*-
import cv2
import os
import numpy as np

class FrameDivision :
    __VIDEO_DIR = 'video/'
    __VIDEO_EXT = '.h264'

    def Frame_Division(self, STEP : str) :

        # Read Video
        cap = cv2.VideoCapture(os.path.join(self.__VIDEO_DIR, STEP + self.__VIDEO_EXT))

        # Frames Array
        frames = []

        while(cap.isOpened()) :

            # Frame Division
            result, frame = cap.read()

            if result :
                frames.append(frame)

            else :
                frames = frames[:32]
                frames = np.array([cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) for frame in frames])
                cap.release()
                cv2.imwrite('./mark/tmp.png', frames[0])
                break

        return frames

if __name__ == '__main__' :
    frame_divider = FrameDivision()
    frame_divider.Frame_Division(input('Input Video Name : '))