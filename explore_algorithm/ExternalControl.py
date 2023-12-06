#!/usr/bin/env python3
# coding: utf-8
import socket
import threading
import time
import subprocess
import WheelControl
import server


class ExternalControlThread(threading.Thread):
    def __init__(self, sock: socket.socket, wheel: WheelControl.Wheel):
        super().__init__()
        self.__socket = sock
        self.__wheel = wheel

    def run(self):
        while True:
            try:
                message_raw, address = self.__socket.recvfrom(1024)
                message = message_raw.decode('utf-8')
            except socket.timeout:
                continue

            if message == 'Restart':
                print('Restart red2.0_main.service!')
                self.__wheel.stop()
                time.sleep(1)
                subprocess.Popen(
                    "sudo systemctl restart red2.0_main.service",
                    shell=True)
            elif message == 'Shutdown':
                print('Start shutdown process')
                server.uploadDisconnect()
                self.__wheel.stop()
                time.sleep(1)
                subprocess.Popen("sudo shutdown -h now", shell=True)
            else:
                continue
