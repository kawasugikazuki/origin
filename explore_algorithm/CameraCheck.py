
# coding: utf-8
import cv2
import datetime
import subprocess
import os

def check_cameras():
    
    result = subprocess.run(["v4l2-ctl --list-devices"], shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    camera_info = result.stdout.decode("utf8")
    
    ceilcamera_key = "mmal service"
    floorcamera_key_list = ["HD Webcam", "HBVCAM 5M-AF"]
    
    camera_info = camera_info.splitlines()

    ceilcamera_device = "none"
    floorcamera_device = "none"


    for index, item in enumerate(camera_info):

        
        if ceilcamera_key in str(item):
            ceilcamera_device = camera_info[index+1].split('/')[-1]


        for key_item in floorcamera_key_list:
            if key_item in str(item):
                floorcamera_device = camera_info[index+1].split('/')[-1]

            #if floorcamera_device != "video0":
            #    floorcamera_device = "different" 

    print("ceilcamera_device : {}".format(ceilcamera_device))
    print("floorcamera_device : {}".format(floorcamera_device))

    return ceilcamera_device, floorcamera_device


if __name__ == "__main__":
    
    print(check_cameras())