import math


class Ackermann:
    def __init__(self, w, l, wheel_r):
        self.w, self.l = w, l
        self.w2 = 0.5*w
        self.wheel_r = wheel_r
        self.arm_wheel_angle = math.atan(self.w2*l)

    def lin_ang_to_steer_spin(self, lin, ang):
        spin = self.wheel_r*lin  # Driven wheel rotational speed
        R = lin/ang  # Turning radius
        th1, th2 = math.atan(self.l/(R-self.w2)), math.atan(self.l/(R+self.w2))
        




