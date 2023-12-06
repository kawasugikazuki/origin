# coding:utf-8
# 任意の距離の同心円状での行動用アルゴリズム（MainAlgorithm.pyベースで切り分け）
# 独自機能のインポート（何をインポートすべきか要確認）
import os
import sys
from socket import socket, AF_INET, SOCK_DGRAM

import LEDcenterFinder  # 引数にステップ数だけ
import server
import pigpio
import WheelControl  # 駆動系のインポート(要追加)
import VL53L0X
import FloorLight
import VL53L0X_multi_example
# 床撮像カメラのインポート（20210822現在プログラムなし）

# python 標準の機能のインポート
import random
from time import sleep
from scipy.stats import norm  # 正規分布


gpio = pigpio.pi()  # GPIOにアクセスするインスタンスを作成
wheel = WheelControl.Wheel(gpio)
light = FloorLight.FloorLED(gpio)

# ～以下、危惧される調整～
# ロボットのプログラムの起動はSSH接続で始めるように
# カメラの座標系がどうなっているのか確認する必要あり（前方が角度0なのか, 147行目あたりのif文の条件に影響）
# 秒数　→　角度の計算を後でやる。具体的には実機が360度回転するのにかかる秒数を実験して、1度あたりにかかる時間を算出してほしい。
# RED2はバックできない設計なので、下がるところは180度回転して戻るように!
# マーカー認識にかかる時間次第で、ロボットの行動を全体的に遅くする必要がある。(sleepや旋回・前進速度の調整)
# 他のプログラムはメインプログラム以下の階層に置くこと。（できればおなじ階層に）

# 探査パラメータ------------------
norm1 = 0.0
norm2 = 0.0
# Boidsモデルで離れすぎたと認識する距離
r = 0.0  # 確率の比
Ran = 0.0  # 0~1の乱数
Ran2 = 0.0
Ran3 = 0.0
DegT = 0.0133333333333333  # 1度回転にかかる秒数
Rot1 = 0.0 #分散中心方向
Rot2 = 90.0 * DegT  # 90度旋回
Rot3 = 0.0
Rot4 = 180 * DegT #180度旋回
D = 0.50 # 同心円判定の幅
Rbe = 0.0  # 直前までいた距離におけるR
Fbe = 1.0  # 直前の直進時間のログ
R = 0.0
R2 =0.0
findCenter = True
Th = 0.0
N = 0
Nc = 0
randTranslateTime = 1
back = 0
i = 0
kikyaku = 0
saitaku = 0
JudgeFlag = True

# 探査パラメータここまで-------------

# UDP受信用ソケット
HOST = ''       # ブロードキャストパケットなので空
PORT = 50000    # 50000番固定
sock = socket(AF_INET, SOCK_DGRAM)
sock.bind((HOST, PORT))

# サーバを初期化
server.initialize()

# 探査開始
while True:
    light.on()  # 底面ライトをオンにする
    print('getStatus Before')
    isExploring, speed, mue, sigma, R_Threshold, Height, Kikyaku = server.getStatus() #R_Thresholdは同心円の距離とする。
    print(isExploring)
    # 30回になったら自動で停止
    if N == 30:
        N = 0
        while isExploring:
            isExploring, _, _, _, _, _, _ = server.getStatus()

    elif N == 15:
        R_Threshold = R_Threshold / 2  # ステップの半分に差し掛かったら同心円を半分の大きさにする。

    # ラジコンモード, 本プログラムでは使わないが残しておく。
    if not isExploring:
        print('raz mode')
        isExploring, _, _, _, _, _, _ = server.getStatus()
        print('ラジコンモードのgetStatusの後です。')
        # if isExploring :
        # wheel.stop()
        # print('ifの中です')
        # continue

        message_raw, address = sock.recvfrom(1024)
        message = message_raw.decode('utf-8')

        if message == 'Forward':
            wheel.forward()
        elif message == 'Stop':
            wheel.stop()
        elif message == 'Right':
            wheel.turn_right()
        elif message == 'Left':
            wheel.turn_left()
        elif message == 'PivotRight':
            wheel.pivot_turn_right()
        elif message == 'PivotLeft':
            wheel.pivot_turn_left()
        else:
            wheel.stop()

    # アルゴリズム探査モード
    if isExploring:
        wheel.stop()
        print('Algorithm Mode')
        # 以後が行動アルゴリズム
        # フローは以下
        # 1.旋回量と移動量を決めて旋回
        # 2.ARマーカーの方向・距離の算出
        # 3a.一定の距離以内（R_Threshold ± 50cm）であれば、同心円上で左右に棄却行動（方向は棄却モード依存）
        # 3b.距離が近い or 遠い場合は同心円に乗るように方向選定して移動（マーカー方向 of 反対方向の選択）

        # 1.直進駆動時間と，旋回駆動時間を範囲付きrandomで生成。
        # randTranslateTime = random.uniform(0.5, 1)  # 短く設定すること。
        randTranslateTime = random.uniform(3, 4)  # 長めに設定すること。
        randRotateTime = random.uniform(0, 4.8)

        # 2.<<Find Leader>>
        try:  # 点滅マーカーなしの場合、例外が発生
            R, Th = LEDcenterFinder.getRTheta(Nc)  # 距離(cm)と角度(deg)を取得
            Nc = Nc + 1  # 取得画像のナンバー
            Rot1 = Th * DegT  # 分散中心方向への角度を秒数化
            back = 0
        except Exception:  # 例外発生時に行う処理（点滅マーカーが見つからなかった)
            print("点滅マーカーなし")
            findCenter = False
            if N == 0:  # ARマーカがなければ最初だけ少しずつ回る。
                randRotateTime = 0.5 #基本的にはこっちのパラメータで
                #randRotateTime = 3  # 1度あたり何秒で旋回できるか求めるときは長めに回して検討する。
                wheel.pivot_turn_left_sec(randRotateTime)
                continue  # 現在のwhile文を抜けて、次のwhile文を実行
            # マーカーを見失った場合の処理（一つ前の位置に戻る。）
            else:
                if back == 0:
                    wheel.pivot_turn_left_sec(Rot4)  # 180度反転
                    wheel.forward_sec(Fbe) #ひとつ前の位置まで直進
                    back = 1
                    continue  # 現在のwhile文を抜けて、次のwhile文を実行
                else:
                    wheel.forward_sec(Fbe) #さらに戻らせる
                    continue
                            
        #Distance, Thetaのデータをサーバに送信
        if findCenter:
            server.uploadDeviceData1(N+1, round(R,1), round(Th,1))
        else:
            server.uploadDeviceData1(N+1, -1, round(Th,1))
            findCenter = True
            
        if N == 0:
            Rbe = R  # 最初のステップ時だけ例外処理
        """
        # 3.メトロポリス法を行うプログラム(確率的に採択・棄却を行う。)
        # 正規分布の生成とパラメータ代入(xは距離)、x:定義域、loc:平均m、scale:標準偏差σ
        norm1 = norm.pdf(Rbe, mue, sigma)  # 一つ前の距離における確率
        norm2 = norm.pdf(R, mue, sigma)  # 現地点の距離における確率

        r = norm2 / norm1  # 確率の比をとる。
        Ran = random.random()  # 0~1乱数生成
        Ran2 = random.random()  # 0~1乱数生成、棄却時の左右行動に利用
        Ran3 = random.random()  # 0~1乱数生成、障害物回避の方向選定に利用

        if back == 1:  # 直前までマーカーが見えていたはずなので戻らせる。パラメータを固定し必ず採択
            r = 1
            R = 0
        """

        # 4.位置と3の確率値から確率的行動決定
        print('中心からの値は')
        print(R)
        print("R_Threshold:")
        print(R_Threshold)
        print("同心円の距離+")
        print(R_Threshold + D)
        print("同心円の距離-")
        print(R_Threshold - D)

        if R > (R_Threshold + D):  # マーカーから遠すぎる場合
            wheel.pivot_turn_left_sec(Rot1)  # マーカー方向に向く
            print('マーカー方向へ向く')
                                
        elif R < (R_Threshold - D):  # マーカーに近すぎる場合
            print("Rot1")
            print(Rot1)
            print('一度マーカー方向へ向く')
            wheel.pivot_turn_left_sec(Rot1)  # いったんマーカー方向に向く
            sleep(0.5)
            print('マーカーから離れる方向へ向く(180度回転)')
            wheel.pivot_turn_left_sec(Rot4)  # 反対方向に向く
        
        else:
            # 同心円状にロボットが存在
            # マーカー方向に対して垂直方向を選出
            """
            R2, Th2 = LEDcenterFinder.getRTheta(Nc)  # 距離(cm)と角度(deg)を取得。
            Nc = Nc + 1  # 取得画像のナンバー
            Rot3 = Th2 * DegT  # 分散中心方向への角度を秒数化（RED2に合わせて要変更）
            print("マーカー方向再検出")
            """

            # 確率的に左右に棄却行動
            if Kikyaku == 'A':
                print('A')
                if 0 <= Ran2 < 0.5:  # 左右等しく移動させる場合
                    wheel.pivot_turn_left_sec(Rot1)  # マーカー方向へ向く
                    wheel.pivot_turn_right_sec(Rot2)  # 90度旋回(右)
                else:
                    wheel.pivot_turn_left_sec(Rot1) # マーカー方向へ向く
                    wheel.pivot_turn_left_sec(Rot2)  # 左回転
            elif Kikyaku == 'B':
                print('B')
                if 0 <= Ran2 < 0.66:  # 3回に2回右に棄却行動させる（より顕著に周回行動）
                    wheel.pivot_turn_left_sec(Rot1)  # マーカー方向へ向く
                    wheel.pivot_turn_right_sec(Rot2)  # 90度旋回(右)
                else:
                    wheel.pivot_turn_left_sec(Rot1) # マーカー方向へ向く
                    wheel.pivot_turn_left_sec(Rot2)  # 左回転
            elif Kikyaku == 'C':
                print('C')
                if 0 <= Ran2 < 0.8:  # 5回に4回右に棄却行動させる（より顕著に周回行動）
                    wheel.pivot_turn_left_sec(Rot1)  # マーカー方向へ向く
                    wheel.pivot_turn_right_sec(Rot2)  # 90度旋回(右)
                else:
                    wheel.pivot_turn_left_sec(Rot1)  # マーカー方向へ向く
                    wheel.pivot_turn_left_sec(Rot2)  # 左回転
            elif Kikyaku == 'D':
                print('D')
                if 0 <= Ran2 < 1:  # すべての棄却行動で右に旋回させる（完全な周回行動)
                    wheel.pivot_turn_left_sec(Rot1)  # マーカー方向へ向く
                    wheel.pivot_turn_right_sec(Rot2)  # 90度旋回(右)
                else:
                    wheel.pivot_turn_left_sec(Rot1) # マーカー方向へ向く
                    wheel.pivot_turn_left_sec(Rot2)  # 左回転
            
        # ここまで旋回行動なので、直進を入れる。
        wheel.forward_sec(randTranslateTime)
        print("直進")
        
        # 障害物センサが問題なくなるまではコメントアウトしておく。
        """
        # 障害物を検知するまで直進
        
        if VL53L0X.VL53L0X.get_distance() < 125:  # 距離センサのレンジは100mm以上にしないと超信地旋回ができない。
            print("障害物検知")
            th90 = 100.0
            Rot2 = th90 * DegT  # 度数から秒数への変換(RED2に合わせて式調整)

            # 障害物回避
            if 0 <= Ran3 < 0.5:
                wheel.pivot_turn_right_sec(Rot2)  # 障害物に対して90度旋回(右)
                wheel.forward_sec(randTranslateTime)
            else:
                wheel.pivot_turn_left_sec(Rot2)  # 左回転
                wheel.forward_sec(randTranslateTime)
            # 回避中に障害物検知したら止まる。
        else:
            print("障害物なし")
        """
        
        # integer increment
        Rbe = R  # 現在の距離の保存

        N = N + 1  # ステップ数
        Fbe = randTranslateTime  # 直進時間を記録

        """
        #採択，棄却，Boidsのデータをサーバに送信
        if Boids == 1:
            server.uploadDeviceData2(N, 0, 0, 1)
        elif JudgeFlag:
            server.uploadDeviceData2(N, 1, 0, Boids)
        else:
            server.uploadDeviceData2(N, 0, 1, Boids)
        """

    else:
        # 探査しないときは一秒待つ
        # sleep(1)
        N = 0
        # ステップ数Nを送信
