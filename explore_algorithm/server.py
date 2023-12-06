# -*- coding: utf-8 -*-

#モジュールが見つからない場合，パスを追加する
import sys
#print(sys.path)

import base64
import json
import paho.mqtt.client as mqtt
import socket
import threading
import time

#制御パラメータ
global data
data={'IsExploring':True, 'TransitTime':2.0, 'Mu':1.0, 'Sigma':1.0, 'Outer_Rth':3.0, 'Inner_Rth':0.0, 'Height':2.0, 'BetweenMarkers':0.8, 'Height_Correction':True, 'Reject':'A', 'MarkerColor':'Green', 'ShutterSpeed':100, 'LeftPWM':0.0,'RightPWM':0.0, 'Xcoord':0.0, 'Ycoord':0.0}

#自身のIPアドレスを探す
def search_my_IPaddress():
    myIP = '';
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8",80))
    myIP = s.getsockname()[0]
    s.close()
    
    return myIP,True

#自身のIPアドレスを獲得
def get_my_IPaddress():
    myip = ""
    end_flag=False
    while not end_flag:
        try:
            myip, end_flag = search_my_IPaddress()
            #print(myip, end_flag)
            if end_flag == True:
                break; 
        except OSError as e:
            print("[Get my IP Error] " + str(e))
            time.sleep(3)
        except Exception as ex:
            print("[Other Error] " + str(ex))
    
    return myip

myip=get_my_IPaddress()

#MQTTクライアントの初期化
def initialize(Broker_IPAddress):
    
    #メッセージが届いたときの処理
    def on_message(client, userdata, msg):
        #msg.topicにトピック名がmsg.payloadに届いたデータ本体が入っている
        #Byte列を変換し，それぞれのバラメータを代入
        #全体制御パラメータ受信
        if msg.topic =="RED/Status":
            global data
            data = json.loads(msg.payload.decode("ascii"))
            print(getStatus())
        #個別制御パラメータ受信
        elif msg.topic == "RED/"+myip+"/Param":
            data = json.loads(msg.payload.decode("ascii"))            
            print(getStatus())

    #ブローカに接続できた時の処理（MQTT）
    def on_connect(client, userdata, flag, rc):
        #接続できた旨表示
        print("connected OK.")
        #管理システム側への通知
        client.publish("RED/"+myip+"/connect","connect RED "+myip,0,True)
        #パラメータの変更に対するトピックをsubする
        client.subscribe("RED/Status",qos=1)
        client.subscribe("RED/"+myip+"/Param",qos=1)
        

    #ブローカが切断したときの処理（MQTT）
    def on_disconnect(client, userdata, flag, rc=-1):
        if rc != 0:
            print("Unexpected disconnection.")
            global data
            data['IsExploring']=False
    
    global client
    client = mqtt.Client("RED(" + myip + ")") #()の中身はclientNameで自分のIPアドレスとした。

    client.reconnect_delay_set(min_delay = 5, max_delay=10)   #再接続時の時間間隔を設定
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    #client側から切断したときにブローカに送られるメッセージ
    client.will_set("RED/"+myip+"/disconnect", "disconnect RED " + myip, 0, True);
    
    try:
        #接続
        #Klab_HUB_2.4
        #管理アプリを開く端末のIPアドレスにすること
        client.connect(Broker_IPAddress, 1883, 60)
        
    except Exception as e:
        print("connection failed")
        print(e)
    
    loop_thread = threading.Thread(target=client.loop_forever)
    loop_thread.setDaemon(True)
    loop_thread.start()

#現在の制御パラメータを獲得
def getStatus():
    return data

#画像をサーバに送信
def uploadImage(imageData, filename, whereImage):
    
    try:        
        #dict -> json形式にする
        Image_dict = {filename:base64.b64encode(imageData).decode('utf-8')}
        Image_json = json.dumps(Image_dict)
        client.publish("RED/" + myip + "/"+whereImage, Image_json, 0)
        
        print("send filename and imageData")

    except Exception as e:
        print("Upload error(image)")
        print(e)

#Distance, Theta, 採択，棄却，Boids, 旋回量のデータをサーバに送信
def uploadDeviceData(Step, R, Th, accept, reject, boids, Rot, TransitTime, color):
    try:
        #dict -> json形式にする
        DataDict = {"Group":"RED", "ID":myip, "Step":str(Step), "Distance":str(R), "Azimuth":str(Th),"TransitTime":str(TransitTime),"Accept":str(accept), "Reject":str(reject), "Boids":str(boids),"RandomRot":str(Rot),"Color":color }
        DataJson = json.dumps(DataDict)
        client.publish("RED/" + myip + "/DeviceData",DataJson, 0)
        print("upload DeviceData")
    
    except Exception as e:
        print("upload error(DeviceData)")
        print(e)

def uploadObstacleInfo(Step, ObstacleFlag_goal, ObstacleFlag_avoidance, CountTime):
    try:
        DataDict = {"Group":"RED", "ID":myip, "Step":Step, "ObstacleFlag_goal":ObstacleFlag_goal, "ObstacleFlag_avoidance":ObstacleFlag_avoidance,
        "CountTime":CountTime}
        DataJson = json.dumps(DataDict)
        client.publish("RED/"+myip+"/Obstacle", DataJson, 0)
        print("upload ObstacleInfo")
    except Exception as e:
        print("upload error(ObstacleInfo)")
        print(e)

def uploadDisconnect():
    try:
        client.publish("RED/"+myip+"/disconnect", "disconnect RED " + myip, 0, True);
    except Exception as e :
        print("upload error(disconnect)")
        print(e)

#ロボットの状態をサーバに送信
def uploadRobotStatus(CeilCamStatus, FloorCamStatus, BatteryStatus):
    try:
        DataDict = {"Group":"RED", "ID":myip, "CeilCam":str(CeilCamStatus), "FloorCam":str(FloorCamStatus), "Battery":str(BatteryStatus)}
        DataJson = json.dumps(DataDict)
        client.publish("RED/"+myip+"/RobotStatus", DataJson, 0)
        print("upload RobotStatus")
    except Exception as e:
        print("upload error(RobotStatus)")
        print(e)
        
if __name__ == "__main__":
    print("input Broker_IPAddress : ")
    initialize(input())
