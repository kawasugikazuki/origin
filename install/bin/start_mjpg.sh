#!/bin/bash
mjpg_streamer -i "./input_uvc.so -f 10 -r 640x360 -d /dev/video0 -y" -o "./output_http.so -w ./www -p 8080"