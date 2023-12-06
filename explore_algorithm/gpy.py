#! /usr/bin/python3
# -*- coding: utf-8 -*-
import smbus
from time import sleep

class GP2Y0E03(object):

    def __init__(self):
        self.bus=smbus.SMBus(1)
        self.register_gpyu=0x5E
        self.register_gpys=0x5F
    
    def read_gpy2(self, address_gpy2):
        self.data=0
        for i in range(1,11):
            #11-4bit data
            self.ue=self.bus.read_word_data(address_gpy2,self.register_gpyu)
            #3-0bit data
            self.shita=self.bus.read_word_data(address_gpy2,self.register_gpys)
            self.ue=self.ue & 0xff
            self.shita=self.shita & 0xff
            self.kobetu =((self.ue*16+self.shita)/16)/4
            self.data = self.data + self.kobetu
            sleep(0.005)
        self.average = self.data/10
        return self.average
            
if __name__ == '__main__':
    X = GP2Y0E03()
    address_left = 0X40
    address_right = 0x50
    while True :
        Value_left = X.read_gpy2(address_left)
        Value_right = X.read_gpy2(address_right)
        print(Value_left,Value_right)
        sleep(0.5)
    
