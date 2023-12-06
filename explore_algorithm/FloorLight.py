#! /usr/bin/env python3
# coding: utf-8
import pigpio
import time


class FloorLight:
    # PWM setting
    __FREQ = 50000
    __RANGE = 100

    # Output elias
    __HIGH = 1
    __LOW = 0

    # PIN
    __PIN = 21
    __PIN2 = 6

    def __init__(self, gpio: pigpio.pi):
        self.__gpio = gpio
        self.__gpio2 = gpio
        self.__gpio.set_mode(self.__PIN, pigpio.OUTPUT)
        self.__gpio.set_PWM_frequency(self.__PIN, self.__FREQ)
        self.__gpio.set_PWM_range(self.__PIN, self.__RANGE)
        self.__gpio2.set_mode(self.__PIN2, pigpio.OUTPUT)
        self.__gpio2.set_PWM_frequency(self.__PIN2, self.__FREQ)
        self.__gpio2.set_PWM_range(self.__PIN2, self.__RANGE)
        self.on()

    def off(self):
        self.__gpio.write(self.__PIN, self.__LOW)
        self.__gpio2.write(self.__PIN2, self.__LOW)

    def on(self):
        self.__gpio.write(self.__PIN, self.__HIGH)
        self.__gpio2.write(self.__PIN2, self.__HIGH)

    def on_pwm(self, duty: int):
        if duty < 0:
            self.off
        elif duty > 100:
            self.on

        self.__gpio.set_PWM_dutycycle(self.__PIN, duty)
        self.__gpio2.set_PWM_dutycycle(self.__PIN2, duty)

    def dimly_blinking(self):
        for i in range(100):
            self.on_pwm(i)
            time.sleep(0.01)
        for i in range(100, 0, -1):
            self.on_pwm(i)
            time.sleep(0.01)


if __name__ == '__main__':
    gpio = pigpio.pi()
    led = FloorLight(gpio)
    led.dimly_blinking()
    led.off()
