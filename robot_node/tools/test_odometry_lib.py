import time
import math
import numpy as np
from math import cos, sin
import matplotlib.pyplot as plt
# from arduino import protocol

class Controller(object):
    def __init__(self):
        self.x_goal = 0
        self.y_goal = 0
        self.pos_error = 1
        self.max_linear_speed = 0
        self.max_angular_speed = 0

        self.prev_time = 0

    def position_controller(self, x, y, theta):

        x_diff = self.x_goal - x
        y_diff = self.y_goal - y
        delta_l = math.sqrt(x_diff**2 + y_diff**2)
        e_alpha = math.atan2(y_diff, x_diff) - theta
        e_rho = delta_l*math.cos(e_alpha)
        self.pos_error = delta_l

        w = e_alpha
        v = e_rho

        if v > self.max_linear_speed:
            v = self.max_linear_speed
        if v < -self.max_linear_speed:
            v = -self.max_linear_speed
        if w > self.max_angular_speed:
            w = self.max_angular_speed
        if w < -self.max_angular_speed:
            w = -self.max_angular_speed

        return v, w


class TestOdometry(object):
    def __init__(self,
                 robot,
                 controller):
        self.robot = robot
        self.controller = controller
        
    def move_to_point_test(self):
        x, y, v, w = [], [], [], []

        log = {'x':[], 'y':[]}

        v.append(0)
        w.append(0)
        
        x, y, th = 0,0,0
        while abs(self.controller.pos_error) > 0.02:
            x, y, yaw, v, w, vr, vl = self.robot.getOdometry()
            
            v, w = self.controller.position_controller(x, 
                                                       y, 
                                                       yaw)
        
            self.robot.setMotors(v, w)

            log['x'].append(x)
            log['y'].append(y)

        self.robot.setMotors(0, 0)
        self.robot.stop()  

        return log  

    def test_speed(self,
                   set_linear,
                   set_angular,
                   full_time_test):
        log = {'time':[], 'vr':[], 'vl':[]}
        prev_time = time.time()
        full_time = 0
        log['time'].append(full_time)
        log['vr'].append(0)
        log['vl'].append(0)
        self.robot.setMotors(set_linear, set_angular)
        while full_time <= full_time_test:
            
            x, y, yaw, v, w, vr, vl = self.robot.getOdometry()

            delta_time = time.time() - prev_time
            prev_time = time.time()
            full_time += delta_time

            log['time'].append(full_time)
            log['vr'].append(vr)
            log['vl'].append(vl)

        self.robot.setMotors(0.0, 0.0)

        return log


class Utils(object):
    def __init__(self):
        pass
    def plot_velocity(self, speed, time, label_str, color):
        
        plt.plot(time, speed, color, label=label_str)

    def plot_trajectory(self, x, y):
        plt.plot(x, y, '-b', label='trajectory')
        plt.legend()
        plt.title('move to goal point test')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.show()
    
    def show_plot_and_legend(self):
        plt.legend()
        plt.title('test speed')
        plt.xlabel('time [sec]')
        plt.ylabel('v [m/s]')
        plt.show()

    def analize_max_min_speed(self, vr, vl):
        print("max speed left: {0}", max(vl))
        print("max speed right: {0}", min(vr))
            
        


    


    
    
    

    

       
                        
