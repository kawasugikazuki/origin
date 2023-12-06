#!/usr/bin/env python3
# coding: utf-8
from typing import Dict


class ExploreParameters:
    def __init__(self) -> None:
        self.is_exploring = False
        self.transit_time = 2.0
        self.mu = 1.0
        self.sigma = 1.0
        self.outer_r_th = 3.0
        self.inner_r_th = 0.0
        self.height = 2.2
        self.between_markers = 0.8
        self.height_correction = True
        self.reject_mode = 'A'
        self.marker_color = 'Green'
        self.shutter_speed = 100
        self.left_pwm_variate = 0.0
        self.right_pwm_variate = 0.0
        self.virtualmarker_x_coord = 0.0
        self.virtualmarker_y_coord = 0.0

    def decode_json_data(self, data: Dict) -> None:
        self.is_exploring = data['IsExploring']
        self.transit_time = data['TransitTime']
        self.mu = data['Mu']
        self.sigma = data['Sigma']
        self.outer_r_th = data['Outer_Rth']
        self.inner_r_th = data['Inner_Rth']
        self.height = data['Height']
        self.between_markers = data['BetweenMarkers']
        self.height_correction = data['Height_Correction']
        self.reject_mode = data['Reject']
        self.marker_color = data['MarkerColor']
        self.shutter_speed = data['ShutterSpeed']
        self.left_pwm_variate = data['LeftPWM']
        self.right_pwm_variate = data['RightPWM']
        self.virtualmarkerxcoord = data['Xcoord']
        self.virtualmarkerycoord = data['Ycoord']
