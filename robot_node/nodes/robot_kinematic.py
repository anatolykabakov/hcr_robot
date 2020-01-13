
import math
import time

class RobotKinematic:
    def __init__(self,
                 wheel_base,
                 wheel_radius):
        self.x = 0
        self.y = 0
        self.yaw=0
        self.prev_time=0
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius

    def update(self, vr, vl):
        #odometry navigation 
        # http://faculty.salina.k-state.edu/tim/robotics_sg/Control/kinematics/unicycle.html
        dt = time.time() - self.prev_time
        self.prev_time = time.time()

        wr = vr/self.wheel_radius
        wl  = vl/self.wheel_radius

        v = (self.wheel_radius/2)*(wr + wl)
        w = (self.wheel_radius/self.wheel_base)*(wr - wl)

        self.yaw+=(w * dt)
        self.yaw = self.normalize_angle(self.yaw)
        self.x += v*math.cos(self.yaw) * dt
        self.y += v*math.sin(self.yaw) * dt

    def normalize_angle(self, angle):
        while (angle > math.pi):
            angle -= 2.0 * math.pi
            
        while (angle < -math.pi):
            angle += 2.0 * math.pi
        return angle
            
