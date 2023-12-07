#!/usr/bin/env python3
# coding: utf-8
import math
from scipy.stats import norm
import random
import socket
import csv
import os

# User defined modules
import server
import ExproleParameters
import FloorImage
import LEDcenterFinder
import WheelControl
import GyroScope
import MesurementData
import MotorControl
import pigpio

class SatoAlgorithm:
    def __init__(self, sock: socket, wheel: WheelControl.Wheel, BrokerIP: str) -> None:
        # インスタンス系
        self.__sock = sock
        self.__wheel = wheel
        self.__gpio = pigpio.pi()
        left_motor = MotorControl.BD6231(self.__gpio, 27, 22)
        right_motor = MotorControl.BD6231(self.__gpio, 19, 26)
        self.__center_finder = LEDcenterFinder.LEDCenterFinder()
        self.gyroscope = GyroScope.GyroScope(self.__gpio, self.__wheel,left_motor,right_motor)
        self.__floorImage = FloorImage.FloorImage()
        self.__exploration_tag = ""
        server.initialize(BrokerIP)

        # 探査パラメータ系
        self.__explore_parameters = ExproleParameters.ExploreParameters()
        self.__step = 0
        self.__accept_num = 0
        self.__reject_num = 0
        self.__image_num = 0
        self.__marker_num = 0
        self.__current_distance_from_center = 0.0
        self.__previous_distance_from_center = 0.0
        self.__current_distance_from_center_at_rejected = 0.0
        self.__azimuth_to_center = 0.0
        self.__azimuth_to_center_at_rejected = 0.0
        self.__x = 0.0
        self.__y = 0.0
        self.__previous_transit_time = 0.0
        self.__is_moved_backward = False
        self.__is_accepted = True
        self.__reject_direction = 'LEFT'
        self.__reject_movement_offset_degree = 0.0
        self.__practical_rotate_angle = 0.0
        self.__practical_transit_time = 0.0
        self.__can_find_center = True
        self.__is_collision_for_goal = 0
        self.__is_collision_for_avoidance = 0
        self.__time_until_collision = 0.0
        
        #ストレス関係
        self.__collision_count_for_right_avoidance = 0
        self.__collision_count_for_left_avoidance = 0
        self.__collision_count_for_goal = 0
        self.__not_collision_count_for_goal = 0
        self.__avoidance_direction = "none"
        self.__left_avoidance_ratio = 0.5
    
        self.__extend_time = 0.0
        
        #計測データの蓄積リスト
        self.mesurement_data_List = []

    def main(self, floorcamera_status, exploration_tag) -> None:
        # MQTTでパラメータを受信
        print('Get status')
        self.__explore_parameters.decode_json_data(server.getStatus())
        self.__exploration_tag = exploration_tag
        print('isExploring: {}'.format(self.__explore_parameters.is_exploring))
        self.__wheel.set_duty_bias(self.__explore_parameters.left_pwm_variate, self.__explore_parameters.right_pwm_variate)
        
        print("left_rot_ratio : ", self.__left_avoidance_ratio)
        
        # FalseかつListにデータがあればcsv保存
        if self.__explore_parameters.is_exploring == False and len(self.mesurement_data_List) > 0:
            folder_path = "./mesurement_data"
            os.makedirs(folder_path, exist_ok=True)
            filename = "/" + self.__exploration_tag +".csv"
            f=open(folder_path + filename, 'w', newline="")
            
            csv_writer = csv.writer(f)
            csv_writer.writerow(["Group","ID","Step", "Height_Correction", "Distance", "Azimuth", "Rotation", "TransitTime", "Inner_Rth", "Mu", "Sigma", "Outer_Rth", "RejectMode", "MarkerColor", "Decision", "Collision_for_goal", "Collision_for_avoidance", "Until_Collision"])
            
            
            
            for index in range(len(self.mesurement_data_List)):
                try:
                    csv_writer.writerow([self.mesurement_data_List[index].group,
                                        self.mesurement_data_List[index].id,
                                        self.mesurement_data_List[index].height_correction,
                                        self.mesurement_data_List[index].step,
                                        self.mesurement_data_List[index].distance,
                                        self.mesurement_data_List[index].azimuth,
                                        self.mesurement_data_List[index].rotation,
                                        self.mesurement_data_List[index].transit_time,
                                        self.mesurement_data_List[index].inner_r_th,
                                        self.mesurement_data_List[index].mu,
                                        self.mesurement_data_List[index].sigma,
                                        self.mesurement_data_List[index].outer_r_th,
                                        self.mesurement_data_List[index].reject_mode,
                                        self.mesurement_data_List[index].marker_color,
                                        self.mesurement_data_List[index].decision,
                                        self.mesurement_data_List[index].collision_for_goal,
                                        self.mesurement_data_List[index].collision_for_avoidance,
                                        self.mesurement_data_List[index].until_collision])
                except:
                    break;
            f.close()
            
            #リスト内の要素を削除
            self.mesurement_data_List.clear()

        # ラジコンモード
        if not self.__explore_parameters.is_exploring:
            print('Radio Control Mode')
            self.__radio_control()

        # アルゴリズム探査モード
        if self.__explore_parameters.is_exploring:
            print('Algorithm exploration mode')
            self.__algorithm_exploration(floorcamera_status)

    def __radio_control(self) -> None:
        self.__explore_parameters.decode_json_data(server.getStatus())

        try:
            message_raw, address = self.__sock.recvfrom(1024)
            message = message_raw.decode('utf-8')
        except socket.timeout:
            return

        if message == 'Forward':
            self.__wheel.forward()
            #self.gyroscope.forward_sec(10)
        elif message == 'Stop':
            self.__wheel.stop()
        elif message == 'Right':
            self.__wheel.turn_right()
        elif message == 'Left':
            self.__wheel.turn_left()
        elif message == 'PivotRight':
            self.__wheel.pivot_turn_right()
        elif message == 'PivotLeft':
            self.__wheel.pivot_turn_left()
        else:
            self.__wheel.stop()

    def __algorithm_exploration(self, floorcamera_status) -> None:
        """
        アルゴリズムのフロー
        1. 旋回量と移動量を決めて旋回
        2. マーカの方向検出
        3. 距離情報から確率を計算(メトロポリス法)
        4． 確率的行動の決定
        """

        print()
        print('----------------------------------------------------')

        # -- 0.5. 転倒検知と復帰
        try:
            self.gyroscope.jugde()
        except GyroScope.Judge as  e:
            print(e)
            self.gyroscope.returning()
            
        if self.__not_collision_count_for_goal > 3:
            self.__reset_stress()

        # -- 1. 直進駆動時間と旋回駆動時間をrandomで生成 -- #
        random_transit_time = random.uniform(self.__explore_parameters.transit_time, self.__explore_parameters.transit_time + 1)
        self.__practical_transit_time = random_transit_time
        random_rotate_angle = random.uniform(0, 360)

        # -- 2. マーカ検出 -- #
        try:
            print('Detect marker')
            marker_color = self.__explore_parameters.marker_color
            height_correction_flag = self.__explore_parameters.height_correction
            distance, azimuth,x,y = self.__center_finder.getRTheta(self.__image_num, marker_color, self.gyroscope.angles, virtualmarkerxcoord=self.__explore_parameters.virtualmarkerxcoord,virtualmarkerycoord=self.__explore_parameters.virtualmarkerycoord, shutter_speed=self.__explore_parameters.shutter_speed, betweenmarker=self.__explore_parameters.between_markers)
            self.__current_distance_from_center = distance
            self.__azimuth_to_center = azimuth
            self.__x = x
            self.__y = y
            self.__image_num += 1
            self.__marker_num = len(distance) #マーカが１つか２つか要素数で判断
            self.__is_moved_backward = False
            
            self.__can_find_center = True
            
        except Exception as e:
            
            # マーカーを見失った時
            self.__can_find_center = False
            print('Could not find the marker')
            print(e)
        
        
        #画像データ取得
        try:
            # 天井画像送信(list[1]が向かうべき探査中心なので、マーカが１つの場合はlist[0]、マーカ２つの場合はlist[1]の情報を送信)
            if self.__marker_num == 1:
                self.__azimuth_to_center_at_rejected = azimuth[0] + 180.0
                ceilImageName = str(round(self.__current_distance_from_center[0], 1)).replace('.', '-') + "_" + str(round(self.__azimuth_to_center[0], 1)).replace('.', '-') + "_" + str(self.__step + 1) + "_" + server.get_my_IPaddress().split('.')[3]
                self.__center_finder.video_cap.frame(ceilImageName + '_ceil', self.__explore_parameters.shutter_speed,self.__exploration_tag)
                server.uploadImage(open(self.__exploration_tag + '-Ceil/' + ceilImageName + '_ceil.jpg', 'rb').read(), ceilImageName + '_ceil.jpg', 'CeilImage')

                print('Distance from center: {} m'.format(self.__current_distance_from_center[0]))
                print('Azimut to center: {} deg'.format(self.__azimuth_to_center[0]))

            else: #天井画像送信、マーカ2つの場合の処理(marker_num == 2)
                self.__azimuth_to_center_at_rejected = azimuth[1] + 180.0
                ceilImageName = str(round(self.__current_distance_from_center[1], 1)).replace('.', '-') + "_" + str(round(self.__azimuth_to_center[1], 1)).replace('.', '-') + "_" + str(self.__step + 1) + "_" + server.get_my_IPaddress().split('.')[3]
                self.__center_finder.video_cap.frame(ceilImageName + '_ceil', self.__explore_parameters.shutter_speed, self.__exploration_tag)
                server.uploadImage(open(self.__exploration_tag + '-Ceil/' + ceilImageName + '_ceil.jpg', 'rb').read(), ceilImageName + '_ceil.jpg', 'CeilImage')

                print('Distance from center: {} m'.format(self.__current_distance_from_center[1]))
                print('Azimut to center: {} deg'.format(self.__azimuth_to_center[1]))
        except Exception as e:
            
            print('Could not take ceil image')
            print(e)
        
        try:
            # 床画像送信
            if floorcamera_status != "none":
                if self.__marker_num == 1:
                    self.__floorImage.UploadImage(self.__floorImage.takeImage(self.__current_distance_from_center[0], self.__step + 1, self.__azimuth_to_center[0], floorcamera_status, self.__exploration_tag), self.__current_distance_from_center[0], self.__step + 1, self.__azimuth_to_center[0])
                else: #マーカ2つの場合、次に向かうべきマーカからの距離・角度値を紐づけ(marker_num == 2)
                    self.__floorImage.UploadImage(self.__floorImage.takeImage(self.__current_distance_from_center[1], self.__step + 1, self.__azimuth_to_center[1], floorcamera_status, self.__exploration_tag), self.__current_distance_from_center[1], self.__step + 1, self.__azimuth_to_center[1])            

        except Exception as e:
            
            print('Could not take floor image')
            print(e)
            
        
        if self.__can_find_center == False:
            
            
            # 例外値としてdistance = -1.0,azimuth = -1.0を送信
            server.uploadDeviceData(self.__step + 1,
                                    -1.0,
                                    -1.0,
                                    0,
                                    0,
                                    0,
                                    180.0,
                                    self.__previous_transit_time,
                                    self.__explore_parameters.marker_color)

            # 開始時点でマーカを見つけられない場合
            if self.__step == 0:
                # TODO: ここの角度を見直す必要があるかもしれない
                # 15度回転する
                self.gyroscope.pivot_turn_left_deg_g(15.0)
                return

            # 途中でマーカを見失った時
            if self.__is_moved_backward:
                # それでも見つからない場合は更に戻る
                self.__is_collision_for_goal, self.__time_until_collision = self.gyroscope.forward_sec(self.__previous_transit_time)
                
                self.__calc_stress_extend_time(self.__is_collision_for_goal)


                # サーバに直進中の情報をアップロードする
                server.uploadObstacleInfo(self.__step + 1, self.__is_collision_for_goal, 0, self.__time_until_collision)
                
                mesurement_data = MesurementData.MesurementData("RED",
                                                                server.get_my_IPaddress(),
                                                                self.__step + 1 ,
                                                                self.__explore_parameters.height_correction,
                                                                -1.0,
                                                                -1.0,
                                                                0.0,
                                                                self.__previous_transit_time,
                                                                self.__explore_parameters.inner_r_th,
                                                                self.__explore_parameters.mu,
                                                                self.__explore_parameters.sigma,
                                                                self.__explore_parameters.outer_r_th,
                                                                self.__explore_parameters.reject_mode,
                                                                self.__explore_parameters.marker_color,
                                                                3,  #cannot find the marker
                                                                self.__is_collision_for_goal,
                                                                0,
                                                                self.__time_until_collision)
                
                self.mesurement_data_List.append(mesurement_data)

                self.__step += 1

                return
            else:
                # 180度旋回して前回の位置へ戻る
                self.__wheel.pivot_turn_left_deg(180.0)

                self.__is_collision_for_goal, self.__time_until_collision = self.gyroscope.forward_sec(self.__previous_transit_time)
                
                self.__calc_stress_extend_time(self.__is_collision_for_goal)

                # サーバに直進中の情報をアップロードする
                server.uploadObstacleInfo(self.__step + 1, self.__is_collision_for_goal, 0, self.__time_until_collision)
                
                mesurement_data = MesurementData.MesurementData("RED",
                                                                server.get_my_IPaddress(),
                                                                self.__step + 1 ,
                                                                self.__explore_parameters.height_correction,
                                                                -1.0,
                                                                -1.0,
                                                                180.0,
                                                                self.__previous_transit_time,
                                                                self.__explore_parameters.inner_r_th,
                                                                self.__explore_parameters.mu,
                                                                self.__explore_parameters.sigma,
                                                                self.__explore_parameters.outer_r_th,
                                                                self.__explore_parameters.reject_mode,
                                                                self.__explore_parameters.marker_color,
                                                                3,  #cannot find the marker
                                                                self.__is_collision_for_goal,
                                                                0,
                                                                self.__time_until_collision)
                
                self.mesurement_data_List.append(mesurement_data)

                self.__step += 1
                self.__is_moved_backward = True

                return

        # -- 3. メトロポリス法の計算を行う部分 -- #
        # 初回のみ前回の位置は今回の位置とする
        if self.__step == 0:
            self.__previous_distance_from_center = self.__current_distance_from_center

        # 前回の位置と今回の位置における存在確率を計算
        previous_existance_probability = norm.pdf(self.__previous_distance_from_center,
                                                  self.__explore_parameters.mu,
                                                  self.__explore_parameters.sigma)

        current_existance_probability = norm.pdf(self.__current_distance_from_center,
                                                 self.__explore_parameters.mu,
                                                 self.__explore_parameters.sigma)

        # 確率の比と探査に使う乱数を取得
        probability_ratio = current_existance_probability / previous_existance_probability
        random_for_explore = random.random()

        # 同心円モード
        if self.__explore_parameters.sigma == 0.01:
            probability_ratio = self.__calc_concentric_circles_mode()

        #回避行動
        #ひとつ前のステップで__is_collision_for_goalが1の場合
        #前方以外の方向に旋回する

        if self.__is_collision_for_goal == 1:
            print('Obstacle Avoidance Action')

            random_for_avoidance = random.random()
            
            print("extend time : ", self.__extend_time)

            #random_rotate_angle = self.__calc_avoidance_rotation(random_for_avoidance)
            random_rotate_angle = self.__calc_stress_rotation(random_for_avoidance, self.__left_avoidance_ratio)

            self.__practical_transit_time = random_transit_time + 2.0 + self.__extend_time
            self.__practical_rotate_angle = random_rotate_angle

            
            # サーバにデータをアップロード
            index = 1 if self.__marker_num==2 else 0
            server.uploadDeviceData(self.__step + 1,
                                        round(self.__current_distance_from_center[index], 1),
                                        round(self.__azimuth_to_center[index], 1),
                                        0,
                                        0,
                                        0,
                                        round(self.__practical_rotate_angle, 1),
                                        round(self.__practical_transit_time, 1),
                                        self.__explore_parameters.marker_color)

            self.__wheel.pivot_turn_left_deg(random_rotate_angle)
            self.__is_collision_for_avoidance, self.__time_until_collision = self.gyroscope.forward_sec(random_transit_time)

            # 直進時間を記録
            self.__previous_transit_time = random_transit_time

            # サーバに直進中の情報をアップロードする
            server.uploadObstacleInfo(self.__step + 1,0, self.__is_collision_for_avoidance, self.__time_until_collision)
            
            self.__calc_stress_left_rot_ratio(self.__practical_rotate_angle, self.__is_collision_for_avoidance)
            
            mesurement_data = MesurementData.MesurementData("RED",
                                                                server.get_my_IPaddress(),
                                                                self.__step + 1 ,
                                                                self.__explore_parameters.height_correction,
                                                                self.__current_distance_from_center[0],
                                                                self.__azimuth_to_center,
                                                                self.__practical_rotate_angle,
                                                                self.__practical_transit_time,
                                                                self.__explore_parameters.inner_r_th,
                                                                self.__explore_parameters.mu,
                                                                self.__explore_parameters.sigma,
                                                                self.__explore_parameters.outer_r_th,
                                                                self.__explore_parameters.reject_mode,
                                                                self.__explore_parameters.marker_color,
                                                                3,  #avoidance behavior          
                                                                0,
                                                                self.__is_collision_for_avoidance,
                                                                self.__time_until_collision)
                
            self.mesurement_data_List.append(mesurement_data)

            self.__is_collision_for_avoidance = 0
            self.__is_collision_for_goal = 0

            self.__step += 1

            self.__print_internal_data()

            return

        # -- 4. 確率的行動の決定 -- #
        # マーカが一つの時の処理、中心からの距離が遠すぎた場合， Boidsモデルによって中心へ向かって長く移動する
        if self.__marker_num == 1: #マーカ数を配列の要素数から判定
            if self.__current_distance_from_center[0] > self.__explore_parameters.outer_r_th:
                print('Move to center by Boids model')
                random_transit_time = random.uniform(self.__explore_parameters.transit_time + 1, self.__explore_parameters.transit_time + 2)
                self.__practical_transit_time = random_transit_time
                self.__practical_rotate_angle = self.__azimuth_to_center[0]
                # サーバにデータをアップロード
                self.__upload_judge_result(True)    # Boidsなので， 引数はTrue

                self.gyroscope.pivot_turn_left_deg_g(self.__azimuth_to_center[0])
                self.__is_collision_for_goal, self.__time_until_collision = self.gyroscope.forward_sec(random_transit_time)

                # 直進時間を記録
                self.__previous_transit_time = random_transit_time
                
                self.__calc_stress_extend_time(self.__is_collision_for_goal)
                
                mesurement_data = MesurementData.MesurementData("RED",
                                                                server.get_my_IPaddress(),
                                                                self.__step + 1 ,
                                                                self.__explore_parameters.height_correction,
                                                                self.__current_distance_from_center[0],
                                                                self.__azimuth_to_center,
                                                                self.__practical_rotate_angle,
                                                                self.__practical_transit_time,
                                                                self.__explore_parameters.inner_r_th,
                                                                self.__explore_parameters.mu,
                                                                self.__explore_parameters.sigma,
                                                                self.__explore_parameters.outer_r_th,
                                                                self.__explore_parameters.reject_mode,
                                                                self.__explore_parameters.marker_color,
                                                                2,  #boids          
                                                                self.__is_collision_for_goal,
                                                                0,
                                                                self.__time_until_collision)
                
                self.mesurement_data_List.append(mesurement_data)

                # サーバに直進中の情報をアップロードする
                server.uploadObstacleInfo(self.__step + 1, self.__is_collision_for_goal, 0, self.__time_until_collision)

                self.__step += 1

                self.__print_internal_data()

                return

            # マーカが一つの時の処理、中心からの距離が近すぎた場合， Boidsモデルによって中心と反対に長く移動する
            elif self.__current_distance_from_center[0] < self.__explore_parameters.inner_r_th:
                print('Stay away from center by Boids model')
                random_transit_time = random.uniform(self.__explore_parameters.transit_time + 1, self.__explore_parameters.transit_time + 2)
                self.__practical_transit_time = random_transit_time
                self.__practical_rotate_angle = self.__azimuth_to_center_at_rejected
                # サーバにデータをアップロード
                self.__upload_judge_result(True)    # Boidsなので， 引数はTrue

                self.gyroscope.pivot_turn_left_deg_g(self.__azimuth_to_center_at_rejected)
                self.__is_collision_for_goal, self.__time_until_collision = self.gyroscope.forward_sec(random_transit_time)

                # 直進時間を記録
                self.__previous_transit_time = random_transit_time
                
                self.__calc_stress_extend_time(self.__is_collision_for_goal)


                # サーバに直進中の情報をアップロードする
                server.uploadObstacleInfo(self.__step + 1, self.__is_collision_for_goal, 0, self.__time_until_collision)
                
                mesurement_data = MesurementData.MesurementData("RED",
                                                                server.get_my_IPaddress(),
                                                                self.__step + 1 ,
                                                                self.__explore_parameters.height_correction,
                                                                self.__current_distance_from_center[0],
                                                                self.__azimuth_to_center,
                                                                self.__practical_rotate_angle,
                                                                self.__practical_transit_time,
                                                                self.__explore_parameters.inner_r_th,
                                                                self.__explore_parameters.mu,
                                                                self.__explore_parameters.sigma,
                                                                self.__explore_parameters.outer_r_th,
                                                                self.__explore_parameters.reject_mode,
                                                                self.__explore_parameters.marker_color,
                                                                2,  #boids  
                                                                self.__is_collision_for_goal,
                                                                0,
                                                                self.__time_until_collision)
                
                self.mesurement_data_List.append(mesurement_data)

                self.__step += 1

                self.__print_internal_data()

                return

        #マーカが二つの場合
        else:
            #前のマーカへの行動処理、前のマーカが次のマーカの間に存在する場合、推定誤差を考慮し判定値に多少幅を用意
            if 0 <= abs(self.__azimuth_to_center[0] - self.__azimuth_to_center[1]) <= 15:
                print('マーカが二つの場合')
                print('Stay away from center by Boids model')
                random_transit_time = random.uniform(self.__explore_parameters.transit_time + 2, self.__explore_parameters.transit_time + 3) #更に多めの移動
                self.__practical_transit_time = random_transit_time
                self.__practical_rotate_angle = self.__azimuth_to_center_at_rejected + 90 # 前のマーカが次のマーカとの間にないように前のマーカ方向に対して鉛直方向へ
                # サーバにデータをアップロード
                self.__upload_judge_result(True)    # Boidsなので， 引数はTrue
                self.gyroscope.pivot_turn_left_deg_g(self.__azimuth_to_center_at_rejected + 90)
                self.__is_collision_for_goal, self.__time_until_collision = self.gyroscope.forward_sec(random_transit_time)

                # 直進時間を記録
                self.__previous_transit_time = random_transit_time
                
                self.__calc_stress_extend_time(self.__is_collision_for_goal)

                # サーバに直進中の情報をアップロードする
                server.uploadObstacleInfo(self.__step + 1, self.__is_collision_for_goal, 0, self.__time_until_collision)
                
                
                mesurement_data = MesurementData.MesurementData("RED",
                                                                server.get_my_IPaddress(),
                                                                self.__step + 1 ,
                                                                self.__explore_parameters.height_correction,
                                                                self.__current_distance_from_center[0],
                                                                self.__azimuth_to_center,
                                                                self.__practical_rotate_angle,
                                                                self.__practical_transit_time,
                                                                self.__explore_parameters.inner_r_th,
                                                                self.__explore_parameters.mu,
                                                                self.__explore_parameters.sigma,
                                                                self.__explore_parameters.outer_r_th,
                                                                self.__explore_parameters.reject_mode,
                                                                self.__explore_parameters.marker_color,
                                                                2,  #boids         
                                                                self.__is_collision_for_goal,
                                                                0,
                                                                self.__time_until_collision)
                
                self.mesurement_data_List.append(mesurement_data)

                self.__step += 1

                self.__print_internal_data()

                return
            # 前のマーカへの行動処理、前のマーカに対して距離が近すぎた場合， Boidsモデルによって前のマーカ方向から離れる方向に長く移動する
            elif self.__current_distance_from_center[0] < self.__explore_parameters.inner_r_th:
                print('Stay away from previous center by Boids model')
                random_transit_time = random.uniform(self.__explore_parameters.transit_time + 1, self.__explore_parameters.transit_time + 2)
                self.__practical_transit_time = random_transit_time
                self.__practical_rotate_angle = self.__azimuth_to_center_at_rejected + 90
                # サーバにデータをアップロード
                self.__upload_judge_result(True)    # Boidsなので， 引数はTrue

                self.gyroscope.pivot_turn_left_deg_g(self.__azimuth_to_center_at_rejected + 90)
                self.__is_collision_for_goal, self.__time_until_collision = self.gyroscope.forward_sec(random_transit_time)

                # 直進時間を記録
                self.__previous_transit_time = random_transit_time
                
                self.__calc_stress_extend_time(self.__is_collision_for_goal)

                # サーバに直進中の情報をアップロードする
                server.uploadObstacleInfo(self.__step + 1, self.__is_collision_for_goal, 0, self.__time_until_collision)
                
                mesurement_data = MesurementData.MesurementData("RED",
                                                                server.get_my_IPaddress(),
                                                                self.__step + 1 ,
                                                                self.__explore_parameters.height_correction,
                                                                self.__current_distance_from_center[0],
                                                                self.__azimuth_to_center,
                                                                self.__practical_rotate_angle,
                                                                self.__practical_transit_time,
                                                                self.__explore_parameters.inner_r_th,
                                                                self.__explore_parameters.mu,
                                                                self.__explore_parameters.sigma,
                                                                self.__explore_parameters.outer_r_th,
                                                                self.__explore_parameters.reject_mode,
                                                                self.__explore_parameters.marker_color,
                                                                2,  #boids         
                                                                self.__is_collision_for_goal,
                                                                0,
                                                                self.__time_until_collision)
                
                self.mesurement_data_List.append(mesurement_data)
                
                self.__step += 1

                self.__print_internal_data()

                return

            #次のマーカへの処理
            elif self.__current_distance_from_center[1] > self.__explore_parameters.outer_r_th:
                print('Move to center by next Boids model')
                random_transit_time = random.uniform(self.__explore_parameters.transit_time + 1, self.__explore_parameters.transit_time + 2)
                self.__practical_transit_time = random_transit_time
                self.__practical_rotate_angle = self.__azimuth_to_center[1]
                # サーバにデータをアップロード
                self.__upload_judge_result(True)    # Boidsなので， 引数はTrue

                self.gyroscope.pivot_turn_left_deg_g(self.__azimuth_to_center[1])
                self.__is_collision_for_goal, self.__time_until_collision = self.gyroscope.forward_sec(random_transit_time)

                # 直進時間を記録
                self.__previous_transit_time = random_transit_time
                
                self.__calc_stress_extend_time(self.__is_collision_for_goal)

                # サーバに直進中の情報をアップロードする
                server.uploadObstacleInfo(self.__step + 1, self.__is_collision_for_goal, 0, self.__time_until_collision)
                
                mesurement_data = MesurementData.MesurementData("RED",
                                                                server.get_my_IPaddress(),
                                                                self.__step + 1 ,
                                                                self.__explore_parameters.height_correction,
                                                                self.__current_distance_from_center[0],
                                                                self.__azimuth_to_center,
                                                                self.__practical_rotate_angle,
                                                                self.__practical_transit_time,
                                                                self.__explore_parameters.inner_r_th,
                                                                self.__explore_parameters.mu,
                                                                self.__explore_parameters.sigma,
                                                                self.__explore_parameters.outer_r_th,
                                                                self.__explore_parameters.reject_mode,
                                                                self.__explore_parameters.marker_color,
                                                                2, #boids         
                                                                self.__is_collision_for_goal,
                                                                0,
                                                                self.__time_until_collision)
                
                self.mesurement_data_List.append(mesurement_data)

                self.__step += 1

                self.__print_internal_data()

                return

            # 次のマーカへの処理、中心からの距離が近すぎた場合， Boidsモデルによって中心と反対に長く移動する
            elif self.__current_distance_from_center[1] < self.__explore_parameters.inner_r_th:
                print('Stay away from next center by Boids model')
                random_transit_time = random.uniform(self.__explore_parameters.transit_time + 1, self.__explore_parameters.transit_time + 2)
                self.__practical_transit_time = random_transit_time
                self.__practical_rotate_angle = self.__azimuth_to_center_at_rejected
                # サーバにデータをアップロード
                self.__upload_judge_result(True)    # Boidsなので， 引数はTrue

                self.gyroscope.pivot_turn_left_deg_g(self.__azimuth_to_center_at_rejected)
                self.__is_collision_for_goal, self.__time_until_collision = self.gyroscope.forward_sec(random_transit_time)

                # 直進時間を記録
                self.__previous_transit_time = random_transit_time

                self.__calc_stress_extend_time(self.__is_collision_for_goal)

                # サーバに直進中の情報をアップロードする
                server.uploadObstacleInfo(self.__step + 1, self.__is_collision_for_goal, 0, self.__time_until_collision)
                
                mesurement_data = MesurementData.MesurementData("RED",
                                                                server.get_my_IPaddress(),
                                                                self.__step + 1 ,
                                                                self.__explore_parameters.height_correction,
                                                                self.__current_distance_from_center[0],
                                                                self.__azimuth_to_center,
                                                                self.__practical_rotate_angle,
                                                                self.__practical_transit_time,
                                                                self.__explore_parameters.inner_r_th,
                                                                self.__explore_parameters.mu,
                                                                self.__explore_parameters.sigma,
                                                                self.__explore_parameters.outer_r_th,
                                                                self.__explore_parameters.reject_mode,
                                                                self.__explore_parameters.marker_color,
                                                                2,  #boids          
                                                                self.__is_collision_for_goal,
                                                                0,
                                                                self.__time_until_collision)
                
                self.mesurement_data_List.append(mesurement_data)

                self.__step += 1

                self.__print_internal_data()

                return

        # 中心からの距離が問題ない場合は， 採択するか棄却するかを判定する
        # 採択の場合
        print('probability_ratio : ' + str(probability_ratio))
        index = 1 if self.__marker_num==2 else 0
        if probability_ratio[index] >= random_for_explore:
            self.__accept_num += 1
            print('Accept!')
            self.__practical_rotate_angle = random_rotate_angle
            # サーバにデータをアップロード
            self.__upload_judge_result(False)   # Boidsではないので， 引数はFalse

            print('Rotate to random direction')
            self.gyroscope.pivot_turn_left_deg_g(random_rotate_angle)
            print('Move forward')
            self.__is_collision_for_goal, self.__time_until_collision = self.gyroscope.forward_sec(random_transit_time)
            self.__is_accepted = True

            # 採択時の距離を保存
            index = 1 if self.__marker_num==2 else 0
            self.__previous_distance_from_center = self.__current_distance_from_center[index]

            # 直進時間を保存
            self.__previous_transit_time = random_transit_time

            self.__calc_stress_extend_time(self.__is_collision_for_goal)

            # サーバに直進中の情報をアップロードする
            server.uploadObstacleInfo(self.__step + 1, self.__is_collision_for_goal, 0, self.__time_until_collision)
            
            mesurement_data = MesurementData.MesurementData("RED",
                                                                server.get_my_IPaddress(),
                                                                self.__step + 1 ,
                                                                self.__explore_parameters.height_correction,
                                                                self.__current_distance_from_center[0],
                                                                self.__azimuth_to_center,
                                                                self.__practical_rotate_angle,
                                                                self.__practical_transit_time,
                                                                self.__explore_parameters.inner_r_th,
                                                                self.__explore_parameters.mu,
                                                                self.__explore_parameters.sigma,
                                                                self.__explore_parameters.outer_r_th,
                                                                self.__explore_parameters.reject_mode,
                                                                self.__explore_parameters.marker_color,
                                                                0,  #accept          
                                                                self.__is_collision_for_goal,
                                                                0,
                                                                self.__time_until_collision)
                
            self.mesurement_data_List.append(mesurement_data)

            self.__step += 1

            self.__print_internal_data()

            return

        # 棄却の場合
        print('Reject at mode {}'.format(self.__explore_parameters.reject_mode))
        self.__reject_num += 1
        self.__is_accepted = False

        """
        NOTE: 前回の位置へ戻ることをやめて， 探査時間を短縮しようとした
              後で復活してもいいように， 記述は残しておく

        # 棄却モードがB以外の場合は一つ前の位置へ移動し， マーカ位置を再取得する
        # if self.__explore_parameters.reject_mode != 'B':
            # print('Move to previous position')
            # self.__wheel.pivot_turn_left_deg(180.0)
            # self.__wheel.forward_sec(self.__previous_transit_time)

            # print("Redetect marker")
            # self.__current_distance_from_center_at_rejected, self.__azimuth_to_center_at_rejected = LEDcenterFinder.getRTheta(self.__image_num)
            # self.__image_num += 1
        """

        # 棄却モードに応じて動作を変える
        if self.__explore_parameters.reject_mode == 'A':
            # 棄却モードA: 左右に等確率で旋回する
            print('Rotate left and right with equal probability')
            self.__reject_mode_A()
        elif self.__explore_parameters.reject_mode == 'B':
            # 棄却モードB: 旋回しない
            print('Do not rotate')
            self.__reject_mode_B()
        elif self.__explore_parameters.reject_mode == 'C':
            # 棄却モードC: 10ステップごとに旋回方向を左右どちらかに切り替える
            print('Change the turning direction once every 10 steps')
            self.__reject_mode_C()
        elif self.__explore_parameters.reject_mode == 'D':
            # 棄却モードD: 毎回右に旋回させる
            self.__reject_mode_D()

        # サーバにデータをアップロードする
        self.__upload_judge_result(False)   # Boidsではないので， 引数はFalse

        # 棄却行動が終わったので直進させる
        if self.__explore_parameters.reject_mode != 'B':
            print('Move forward')
            self.__is_collision_for_goal, self.__time_until_collision = self.gyroscope.forward_sec(random_transit_time)

        # 前回の位置を保存
        self.__previous_distance_from_center = self.__current_distance_from_center

        # 直進時間を記録
        self.__previous_transit_time = random_transit_time

        self.__calc_stress_extend_time(self.__is_collision_for_goal)

        # サーバに直進中の情報をアップロードする
        server.uploadObstacleInfo(self.__step + 1, self.__is_collision_for_goal, 0, self.__time_until_collision)
        
        mesurement_data = MesurementData.MesurementData("RED",
                                                                server.get_my_IPaddress(),
                                                                self.__step + 1 ,
                                                                self.__explore_parameters.height_correction,
                                                                self.__current_distance_from_center[0],
                                                                self.__azimuth_to_center,
                                                                self.__practical_rotate_angle,
                                                                self.__practical_transit_time,
                                                                self.__explore_parameters.inner_r_th,
                                                                self.__explore_parameters.mu,
                                                                self.__explore_parameters.sigma,
                                                                self.__explore_parameters.outer_r_th,
                                                                self.__explore_parameters.reject_mode,
                                                                self.__explore_parameters.marker_color,
                                                                1,  #reject          
                                                                self.__is_collision_for_goal,
                                                                0,
                                                                self.__time_until_collision)
                
        self.mesurement_data_List.append(mesurement_data)

        self.__step += 1

        self.__print_internal_data()

        return

    def __calc_avoidance_rotation(self, rand :float) -> float:
        #tmp_value = math.sqrt(abs(rand - 0.5)*math.pow(math.pi,2)*0.5)
        tmp_value = ((math.pi**3)*0.25*abs(rand - 0.5))**(1/3)

        if rand>=0.5:
            return_value =  math.degrees(math.pi + tmp_value)
        else:
            return_value =  math.degrees(math.pi - tmp_value)

        return_value = round(return_value,1)

        return return_value
    
    def __calc_stress_rotation(self, rand:float, left_direction_ratio:float) -> float:
        random_for_rotation = random.random()
        tmp_value = math.pi * 0.5 * (random_for_rotation)**(1/3)
        
        #左方向旋回
        if rand <= left_direction_ratio:
            return_value = math.degrees(math.pi - tmp_value)
            
        #右方向旋回
        else:
            return_value = math.degrees(math.pi + tmp_value)
            
        return_value = round(return_value,1)
        
        return return_value
    
    def __calc_stress_left_rot_ratio(self, random_rotate_angle:float, collision_flag_for_avoidance):
        
        if 90.0<=random_rotate_angle<=180.0 and collision_flag_for_avoidance==1:
            #左側旋回で衝突した場合
            self.__collision_count_for_left_avoidance += 1
            self.__left_avoidance_ratio -=0.1 if self.__left_avoidance_ratio!=0 else 0
        elif 90.0<=random_rotate_angle<=180.0 and collision_flag_for_avoidance==0:
            #左側旋回で衝突しない場合、左側旋回の確率を上げる
            self.__left_avoidance_ratio +=0.1 if self.__left_avoidance_ratio!=1.0 else 0
            
        
        if 180.0<=random_rotate_angle<=270.0 and collision_flag_for_avoidance==1:
            #右側旋回で衝突した場合
            self.__collision_count_for_right_avoidance += 1
            self.__left_avoidance_ratio +=0.1 if self.__left_avoidance_ratio!=1.0 else 0
        if 180.0<=random_rotate_angle<=270.0 and collision_flag_for_avoidance==0:
            #右側旋回で衝突しない場合、右側旋回の確率を上げる
            self.__left_avoidance_ratio -=0.1 if self.__left_avoidance_ratio!=0 else 0
            
    def __calc_stress_extend_time(self, collision_flag_for_goal):
        
        if collision_flag_for_goal == 1:
            #進みたい方向での衝突が増えると回避行動時の直進も増える
            self.__extend_time += 0.5 if self.__extend_time<=5.0 else 0 
            self.__not_collision_count_for_goal = 0
        
        else:
            self.__not_collision_count_for_goal += 1
            
    def __reset_stress(self):
        print("Reset Stress")
        self.__collision_count_for_goal = 0
        self.__not_collision_count_for_goal = 0
        self.__avoidance_direction = "none"
        self.__collision_count_for_left_avoidance = 0
        self.__collision_count_for_right_avoidance = 0
        self.__left_avoidance_ratio = 0.5
        self.__extend_time = 0.0


    # 同心円モード
    def __calc_concentric_circles_mode(self) -> float:
        #  変数名が長いので， 短い変数に一旦入れる
        mu = self.__explore_parameters.mu
        previous_R = self.__previous_distance_from_center
        current_R = self.__current_distance_from_center

        if previous_R < current_R < mu:
            return 1.0
        elif mu < current_R < previous_R:
            return 1.0
        else:
            return 0.0

    def __reject_mode_A(self) -> None:
        # 左右等確率で棄却行動をする
        random_for_reject_direction = random.random()
        index = 1 if self.__marker_num==2 else 0
        # 同心円の内側
        if self.__explore_parameters.mu <= self.__current_distance_from_center[index]:
            if 0 <= random_for_reject_direction < 0.5:
                # 右旋回
                self.__reject_to_right_inside_circle()
            else:
                # 左旋回
                self.__reject_to_left_inside_circle()
        # 同心円の外側
        else:
            if 0 <= random_for_reject_direction < 0.5:
                # 右旋回
                self.__reject_to_right_outside_circle()
            else:
                # 左旋回
                self.__reject_to_left_outside_circle()

    def __reject_mode_B(self) -> None:
        # 何もしない
        return

    def __reject_mode_C(self) -> None:
        index = 1 if self.__marker_num==2 else 0
        # 10ステップ毎に棄却方向を切り替え
        if self.__step % 10 == 0:
            if self.__reject_direction == 'LEFT':
                self.__reject_direction = 'RIGHT'
                print('Change reject direction: LEFT --> RIGHT')
            else:
                self.__reject_direction = 'LEFT'
                print('Change reject direction: RIGHT --> LEFT')

        # 左右の指定に合わせて， 同心円内外で旋回方向を変更
        if self.__reject_direction == 'LEFT':
            if self.__explore_parameters.mu <= self.__current_distance_from_center[index]:
                self.__reject_to_left_outside_circle()
            else:
                self.__reject_to_left_outside_circle()
        elif self.__reject_direction == 'RIGHT':
            if self.__explore_parameters.mu <= self.__current_distance_from_center[index]:
                self.__reject_to_right_inside_circle()
            else:
                self.__reject_to_right_inside_circle()

    def __reject_mode_D(self) -> None:
        index = 1 if self.__marker_num==2 else 0
        # すべての場合において， 右旋回を行う
        if self.__explore_parameters.mu <= self.__current_distance_from_center[index]:
            self.__reject_to_right_inside_circle()
        else:
            self.__reject_to_right_inside_circle()

    def __reject_to_right_outside_circle(self) -> None:
        index = 1 if self.__marker_num==2 else 0
        azimuth = self.__azimuth_to_center[index]
        offset = self.__reject_movement_offset_degree
        # 負の値が出た場合には旋回方向を変えて対応
        if (90.0 - azimuth) > 0:
            self.__practical_rotate_angle = 360.0 - (90.0 - azimuth - offset)
            self.gyroscope.pivot_turn_right_deg_g(90.0 - azimuth - offset)
        else:
            self.__practical_rotate_angle = azimuth - 90.0 + offset
            self.gyroscope.pivot_turn_left_deg_g(azimuth - 90.0 + offset)

    def __reject_to_left_outside_circle(self) -> None:
        index = 1 if self.__marker_num==2 else 0
        azimuth = self.__azimuth_to_center[index]
        offset = self.__reject_movement_offset_degree
        self.__practical_rotate_angle = 90.0 + azimuth + offset
        self.gyroscope.pivot_turn_left_deg_g(90.0 + azimuth + offset)

    def __reject_to_right_inside_circle(self) -> None:
        index = 1 if self.__marker_num==2 else 0
        azimuth = self.__azimuth_to_center[index]
        offset = self.__reject_movement_offset_degree
        # 負の値が出た場合には旋回方向を変えて対応
        if (90.0 - azimuth) > 0:
            self.__practical_rotate_angle = 360.0 - (90.0 - azimuth + offset)
            self.gyroscope.pivot_turn_right_deg_g(90.0 - azimuth + offset)
        else:
            self.__practical_rotate_angle = azimuth - 90.0 - offset
            self.gyroscope.pivot_turn_left_deg_g(azimuth - 90.0 - offset)

    def __reject_to_left_inside_circle(self) -> None:
        index = 1 if self.__marker_num==2 else 0
        azimuth = self.__azimuth_to_center[index]
        offset = self.__reject_movement_offset_degree
        self.__practical_rotate_angle = 90.0 + azimuth - offset
        self.gyroscope.pivot_turn_left_deg_g(90.0 + azimuth - offset)

    def __upload_judge_result(self, is_boids: bool) -> None:
        boids = 1 if is_boids else 0
        index = 1 if self.__marker_num==2 else 0

        if self.__can_find_center:
            if boids == 1:
                server.uploadDeviceData(self.__step + 1,
                                        round(self.__current_distance_from_center[index], 1),
                                        round(self.__azimuth_to_center[index], 1), 0, 0, boids,
                                        round(self.__practical_rotate_angle, 1),
                                        round(self.__practical_transit_time, 1),
                                        self.__explore_parameters.marker_color)
            elif self.__is_accepted:
                server.uploadDeviceData(self.__step + 1,
                                        round(self.__current_distance_from_center[index], 1),
                                        round(self.__azimuth_to_center[index], 1), 1, 0, boids,
                                        round(self.__practical_rotate_angle, 1),
                                        round(self.__practical_transit_time, 1),
                                        self.__explore_parameters.marker_color)
            else:
                server.uploadDeviceData(self.__step + 1,
                                        round(self.__current_distance_from_center[index], 1),
                                        round(self.__azimuth_to_center[index], 1),
                                        0,
                                        1,
                                        boids,
                                        round(self.__practical_rotate_angle, 1),
                                        round(self.__practical_transit_time, 1),
                                        self.__explore_parameters.marker_color)

    def __print_internal_data(self) -> None:
        # 棄却率と採択率の計算
        if self.__accept_num != 0 or self.__reject_num != 0:   # どちらも0だと， 0除算になるので回避
            accept_ratio = self.__accept_num / (self.__accept_num + self.__reject_num) * 100
            reject_ratio = self.__reject_num / (self.__accept_num + self.__reject_num) * 100
        else:
            accept_ratio = 0
            reject_ratio = 0

        print('~~~ Internal Data ~~~')
        print('Current step: {}'.format(self.__step))
        print('Accept ratio: {} %'.format(accept_ratio))
        print('Reject ratio: {} %'.format(reject_ratio))
        print('Reject mode: {}'.format(self.__explore_parameters.reject_mode))
        if self.__explore_parameters.reject_mode == 'C':
            print('Reject direction: {}'.format(self.__reject_direction))

    def __del__(self):
        self.__wheel.stop_gently()
