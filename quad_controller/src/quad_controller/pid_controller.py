# Copyright (C) 2017 Electric Movement Inc.
# All Rights Reserved.

# Author: Brandon Kinman

#kp=17.0,ki=3.0,kd=10.0, ki_max=10.0

class PIDController:
    def __init__(self, kp = 0.0, ki = 0.0, kd = 0.0, max_windup = 10):
        
        #TODO
        self.kp_ = float(kp)
        self.ki_ = float(ki)
        self.kd_ = float(kd)

        self.max_windup_ = float(max_windup)

        self.set_point_ = 0.0

        self.last_timestamp_ = 0.0

        self.error_sum_ = 0.0
        self.last_error_ = 0.0

        self.u_p = [0]
        self.u_d = [0]
        self.u_i = [0]

    def reset(self):
        #TODO
        self.kp_ = 0.0
        self.kd_ = 0.0
        self.ki_ = 0.0
        self.set_point_ = 0.0
        #self.max_windup_ = 0.0
        self.last_timestamp_ = 0.0
        self.error_sum_ = 0.0
        self.last_error_ = 0.0

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

        delta_time = timestamp - self.last_timestamp_
        if delta_time == 0.0:
        	return 0

        error = self.set_point_ - measured_value

        p = self.kp_*error
        
        self.error_sum_ += error*delta_time
        if self.error_sum_>self.max_windup_:
        	self.error_sum_ = self.max_windup_
        i = self.ki_*self.error_sum_
        

        error_diff = (error - self.last_error_)/delta_time
        d = self.kd_*error_diff

        u = p + i + d

        self.last_timestamp_ = timestamp
        self.last_error_ = error
        
        self.u_p.append(p)
        self.u_i.append(i)
        self.u_d.append(d)

        return u

