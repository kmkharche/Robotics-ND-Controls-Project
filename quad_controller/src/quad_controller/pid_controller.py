# Copyright (C) 2017 Electric Movement Inc.
# All Rights Reserved.

# Author: Brandon Kinman


class PIDController:
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, max_windup = 10):
        
        #TODO
        self.kp_ = float(kp)
        self.ki_ = float(ki)
        self.kd_ = float(kd)

        self.max_windup_ = float(max_windup)

        self.set_point_ = 0.0

        self.last_timestamp_ = 0.0
    
    def reset(self):
        #TODO
        self.kp_ = 0.0
        self.kd_ = 0.0
        self.ki_ = 0.0
        self.set_point_ = 0.0
        self.max_windup_ = 0.0

    def setTarget(self, target):
        #TODO
        self.set_point_ = float(target)

    def setKP(self, kp):
        #TODO
        self.kp_ = float(kp)

    def setKI(self, ki):
        #TODO
        self.ki_ = float(ki)

    def setKD(self, kd):
        #TODO
        self.kd_ = float(kd)

    def setMaxWindup(self, max_windup):
        #TODO
        self.max_windup_ = float(max_windup)

    def update(self, measured_value, timestamp):
        #TODO
        self.last_timestamp_ = timestamp


