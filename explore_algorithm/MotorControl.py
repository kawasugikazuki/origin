#! /usr/bin/env python3
# coding: utf-8
import pigpio
import time


class BD6231:
    # PWM setting
    __FREQ = 50000
    __RANGE = 100
    __DUTY = 70

    # Output elias
    __HIGH = 1
    __LOW = 0

    def __init__(self, gpio: pigpio.pi, fin: int, rin: int):
        self.gpio = gpio
        self.fin = fin
        self.rin = rin
        self.__duty_bias = 0
        self.gpio.set_mode(self.fin, pigpio.OUTPUT)
        self.gpio.set_mode(self.rin, pigpio.OUTPUT)
        self.gpio.set_PWM_frequency(self.fin, self.__FREQ)
        self.gpio.set_PWM_frequency(self.rin, self.__FREQ)
        self.gpio.set_PWM_range(self.fin, self.__RANGE)
        self.gpio.set_PWM_range(self.rin, self.__RANGE)
        self.stop_rotation()

    def set_duty_bias(self, bias: int):
        self.__duty_bias = bias

    def stop_rotation(self):
        self.gpio.write(self.fin, self.__LOW)
        self.gpio.write(self.rin, self.__LOW)

    def break_rotation(self):
        self.gpio.write(self.fin, self.__HIGH)
        self.gpio.write(self.rin, self.__HIGH)

    def forward_rotation(self):
        if self.__DUTY + self.__duty_bias > 100:
            self.gpio.set_PWM_dutycycle(self.fin, 100)
        else:
            self.gpio.set_PWM_dutycycle(self.fin, self.__DUTY + self.__duty_bias)
        self.gpio.write(self.rin, self.__LOW)

    def forward_rotation_pwm(self, duty: int):
        if duty + self.__duty_bias > 100:
            self.gpio.set_PWM_dutycycle(self.fin, 100)
        elif duty < 70:
            self.gpio.set_PWM_dutycycle(self.fin, duty)
        else:
            self.gpio.set_PWM_dutycycle(self.fin, duty + self.__duty_bias)
        self.gpio.write(self.rin, self.__LOW)

    def backward_rotation(self):
        self.gpio.write(self.fin, self.__LOW)
        if self.__DUTY + self.__duty_bias > 100:
            self.gpio.set_PWM_dutycycle(self.rin, 100)
        else:
            self.gpio.set_PWM_dutycycle(self.rin, self.__DUTY + self.__duty_bias)

    def backward_rotation_pwm(self, duty: int):
        self.gpio.write(self.fin, self.__LOW)
        if duty + self.__duty_bias > 100:
            self.gpio.set_PWM_dutycycle(self.rin, 100)
        elif duty < 70:
            self.gpio.set_PWM_dutycycle(self.rin, duty)
        else:
            self.gpio.set_PWM_dutycycle(self.rin, duty + self.__duty_bias)


if __name__ == '__main__':
    # Test code
    motor = BD6231(27, 22)
    motor.forward_rotation()
    time.sleep(5.0)
    motor.backward_rotation()
    time.sleep(5.0)
    motor.break_rotation()
    motor.stop_rotation()
    time.sleep(1.0)
