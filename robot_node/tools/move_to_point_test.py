import time
import math
ERROR_POS = 0.1
import numpy as np
from math import cos, sin
from arduino import protocol
class Controller(object):
    def __init__(self, v, w):
        self.v = v
        self.w = w
        self.x_ref = 0
        self.y_ref = 0
        self.pos_error = 0

    def position_controller(self, x_goal, y_goal, x, y, theta, delta_time):
        x_diff = x_goal - x
        y_diff = y_goal - y
        delta_l = math.sqrt(x_diff**2 + y_diff**2)
        e_alpha = math.atan2(y_diff, x_diff) - theta
        e_rho = delta_l*math.cos(e_alpha)
        self.pos_error = delta_l

        self.w = e_alpha#/delta_time
        self.v = e_rho
        if self.v > 0.2:
            self.v = 0.2
        if self.v < -0.2:
            self.v = -0.2
        if self.w > 1:
            self.w = 1
        if self.w < -1:
            self.w = -1
        if delta_l < ERROR_POS:
            self.w = 0
            self.v = 0

def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].
    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle               
        
if __name__ == '__main__':
    #----Init state--
    robot = protocol('/dev/ttyACM0', 57600)
    controller = Controller(v=0, w=0)
    x = []
    y = []
    v = []
    w = []
    v.append(controller.v)
    w.append(controller.w)
    x_ref = 1
    y_ref = 2
    prev_time = 0
    current_time = 0
    delta_time = 0
    x, y, th = 0,0,0
    wheel_radius = 0.682
    wheel_dist = 0.29

    controller.pos_error = 1
    
    while abs(controller.pos_error) > 0.1:
        x, y, yaw, v, w, vr, vl = robot.getOdometry()
        
        prev_time = current_time
        current_time = time.time()
        delta_time = current_time - prev_time
        
        controller.position_controller(x_ref, y_ref, x, y, yaw, delta_time)
    
        robot.setMotors(round(controller.v,1), round(controller.w,1))
    robot.setMotors(0, 0)
    robot.stop()

       
                        
