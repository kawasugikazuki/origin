#! /usr/bin/env python3
# coding: utf-8
import pigpio
import time
import MotorControl
import FloorLight
import DistanceSensor


class Wheel:
    __sec_per_deg = 1.12 / 360.0

    def __init__(self, gpio: pigpio.pi):
        self.__gpio = gpio
        self.__distance_sensor = DistanceSensor.DistanceSensor(self.__gpio)
        self.__left_motor = MotorControl.BD6231(self.__gpio, 27, 22)
        self.__right_motor = MotorControl.BD6231(self.__gpio, 19, 26)
        self.__find_obstacle_twice = False

    def set_duty_bias(self, left_bias: int, right_bias: int) -> None:
        self.__left_motor.set_duty_bias(left_bias)
        self.__right_motor.set_duty_bias(right_bias)

    def stop(self) -> None:
        self.__left_motor.break_rotation()
        self.__right_motor.break_rotation()
        self.__left_motor.stop_rotation()
        self.__right_motor.stop_rotation()

    def stop_gently(self) -> None:
        # DUTYが40 %くらいでほぼすすまないから一旦これくらいで
        for i in range(7, 3, -1):
            self.forward_pwm(i * 10)
            time.sleep(0.5)
        self.stop()

    def forward(self) -> None:
        self.__left_motor.backward_rotation()
        self.__right_motor.forward_rotation()

    def forward_pwm(self, duty: int):
        self.__left_motor.backward_rotation_pwm(duty)
        self.__right_motor.forward_rotation_pwm(duty)

    def forward_sec(self, seconds: float):
        loop_count = seconds / 0.1
        time_start = time.perf_counter()
        obstacle_flag = 0
        forward_time = 0
        for i in range(int(loop_count / 2)):
            try:
                # 障害物が無いか確認
                # 細かくループを刻むことで，擬似的にthreadingのような動作を行う
                self.__distance_sensor.obstacle_monitoring()
                self.forward()
                time.sleep(0.1)
            except DistanceSensor.FindObstacle:
                if not self.__find_obstacle_twice:
                    self.__find_obstacle_twice = True
                    continue
                print("Find obstacle!")
                time_end = time.perf_counter()
                forward_time = time_end - time_start
                obstacle_flag = 1
                self.__find_obstacle_twice = False
                self.stop_gently()
                """
                # ランダムで左か右を向く
                if random.random() < 0.5:
                    self.pivot_turn_left_deg(90)
                else:
                    self.pivot_turn_right_deg(90)
                    break
                # ここまで
                """

                # 少し戻す場合
                self.turn_right_sec(0.5)
                self.stop()
                self.turn_left_sec(0.5)
                self.stop()
                time.sleep(0.5)
                break
                # ここまで
        
        if obstacle_flag == 0:
            self.stop_gently()
            time_end = time.perf_counter()
            forward_time = time_end - time_start

        return obstacle_flag, forward_time

    def turn_right(self) -> None:
        self.__left_motor.stop_rotation()
        self.__right_motor.backward_rotation()

    def turn_right_sec(self, seconds: float) -> None:
        self.turn_left()
        time.sleep(seconds)
        self.stop()

    def turn_left(self) -> None:
        self.__left_motor.forward_rotation()
        self.__right_motor.stop_rotation()

    def turn_left_sec(self, seconds: float) -> None:
        self.turn_right()
        time.sleep(seconds)
        self.stop()

    def pivot_turn_right(self) -> None:
        self.__left_motor.backward_rotation()
        self.__right_motor.backward_rotation()

    def pivot_turn_right_sec(self, seconds: float) -> None:
        self.pivot_turn_left()
        time.sleep(seconds)
        self.stop()

    def pivot_turn_right_deg(self, degree: float) -> None:
        turn_time = degree * self.__sec_per_deg
        self.pivot_turn_left_sec(turn_time)

    def pivot_turn_left(self) -> None:
        self.__left_motor.forward_rotation()
        self.__right_motor.forward_rotation()

    def pivot_turn_left_sec(self, seconds: float) -> None:
        self.pivot_turn_right()
        time.sleep(seconds)
        self.stop()

    def pivot_turn_left_deg(self, degree: float) -> None:
        turn_time = degree * self.__sec_per_deg
        self.pivot_turn_right_sec(turn_time)


if __name__ == '__main__':
    # Test Code
    gpio = pigpio.pi()
    wheel = Wheel(gpio)
    light = FloorLight.FloorLight(gpio)
    light.on()
    print(wheel.forward_sec(5.0))
    light.off()
