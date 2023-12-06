#!/usr/bin/env python3
# coding: utf-8
import pigpio
import time
import FloorLight


class BatteryManager:
    def __init__(self, gpio: pigpio.pi, led: FloorLight.FloorLight) -> None:
        self.gpio = gpio
        self.led = led

        # Initialize GPIO5
        gpio.set_mode(5, pigpio.INPUT)
        gpio.set_pull_up_down(5, pigpio.PUD_UP)

        # Starting sign
        self.led.on()
        time.sleep(0.1)
        self.led.off()
        time.sleep(0.1)
        self.led.on()
        time.sleep(0.1)
        self.led.off()
        time.sleep(0.1)

    def check_voltage(self) -> bool:
        if self.gpio.read(5):
            return True
        else:
            return False

    def battery_change_sign(self) -> None:
        self.led.on()
        time.sleep(0.1)
        self.led.off()
        time.sleep(0.05)


if __name__ == "__main__":
    gpio = pigpio.pi()
    led = FloorLight.FloorLight(gpio)
    battery = BatteryManager(gpio, led)
    if not battery.check_voltage():
        battery.battery_change_sign()
