#! /usr/bin/env python3
# coding: utf-8
import sys
import socket
import time
import pigpio

import SatoAlgorithm
import FloorLight
import BatteryCheck
import WheelControl
import ExternalControl
import server
import CameraCheck


def main():
    # UDP受信用のソケット
    HOST = ''       # ブロードキャストパケットなので空
    PORT = 50000    # 50000番固定
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    sock.settimeout(0.5)

    gpio = pigpio.pi()
    wheel = WheelControl.Wheel(gpio)
    light = FloorLight.FloorLight(gpio)
    battery = BatteryCheck.BatteryManager(gpio, light)

    # Restart用UDPClient
    UDPClient("", 50001, wheel)
    # Shutdown用UDPClient
    UDPClient("", 50002, wheel)

    # search Broker IPAdress using UDP
    BrokerIP, Exploration_Tag = search(light)

    explore_algorithm = SatoAlgorithm.SatoAlgorithm(sock, wheel, BrokerIP)

    CeilCamera_status = ""
    FloorCamera_status = ""
    
    CeilCamera_status, FloorCamera_status = CameraCheck.check_cameras()
    # カメラ、バッテリ情報をサーバに送信
    server.uploadRobotStatus(CeilCamera_status, FloorCamera_status,"5")

    # アルゴリズムを起動命令をまつ
    while True:
        light.dimly_blinking()
        print('Wait for start command')
        # バッテリー電圧を監視
        if not battery.check_voltage():
            print('Battery voltage is low')
            battery.battery_change_sign()

        # 受信したUDPパケットを解析
        try:
            message_raw, address = sock.recvfrom(1024)
            message = message_raw.decode('utf-8').rstrip('\n')
            print(message)
            # TODO: 送受信するデータを定義する
            if message == 'StartExplore':
                # 受信したら送信元に返信して， ループを抜ける
                print('Start Exploring!')
                print(address)
                sock.sendto(
                    'Success to launch explore algorithm'.encode('utf-8'),
                    address)
                break
            else:
                time.sleep(1.0)
                continue
        except socket.timeout:
            continue

    # 底面ライトを点灯させる
    light.on()

    # ジャイロThred開始
    explore_algorithm.gyroscope.thread()

    while True:
        
        Battery_status = "5"
        
        # カメラ接続を監視
        CeilCamera_status, FloorCamera_status = CameraCheck.check_cameras()
        
        # バッテリー電圧を監視
        if not battery.check_voltage():
            print('Battery voltage is low')
            Battery_status = "warning"
            battery.battery_change_sign()

        # カメラ、バッテリ情報をサーバに送信
        server.uploadRobotStatus(CeilCamera_status, FloorCamera_status, Battery_status)

        # アルゴリズムを実行する
        explore_algorithm.main(FloorCamera_status, Exploration_Tag)


def UDPClient(HOST, PORT, wheel):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    sock.settimeout(0.5)
    external_control_thread = ExternalControl.ExternalControlThread(sock, wheel)
    external_control_thread.start()


# ブローカのIPアドレスを返す
def search(light):

    # UDP受信用のソケット
    HOST = ''       # ブロードキャストパケットなので空
    PORT = 50003    # 50003番固定
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))
    sock.settimeout(0.5)

    # アルゴリズムを起動命令をまつ
    while True:
        print('Wait for Broker IP')
        light.dimly_blinking()
        # 受信したUDPパケットを解析
        try:
            message_raw, address = sock.recvfrom(1024)
            message = message_raw.decode('utf-8').rstrip('\n')
            print(message)
            # TODO: 送受信するデータを定義する
            if "BrokerIP_is_" in message:
                # 受信したら送信元に返信して， ループを抜ける
                print('Catch Broker IP!')
                print(address)
                print(message)
                Broker_IP = message.split('_')[2]
                exploration_tag = message.split('_')[3]
                sock.sendto(
                    'Success to launch BrokerIP'.encode('utf-8'),
                    address)
                return Broker_IP, exploration_tag
            else:
                time.sleep(1.0)
                continue
        except socket.timeout:
            continue


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("KeyboardInterrupt: stopped by keyboard input (ctrl-C)")
        sys.exit()
