import time
import matplotlib.pyplot as plt
from arduino import protocol


if __name__ == '__main__':
    file = open('log_vel_nopid.txt', 'w')

    robot = protocol('/dev/ttyACM0', 57600)
    prev_time = time.time()
    full_time = 0
    log = {'time':[], 'vr':[], 'vl':[]}

    set_linear = 0.3
    set_angular = 0.0
    robot.setMotors(set_linear, set_angular)
    while full_time <= 2:
        
        x, y, yaw, v, w, vr, vl = robot.getOdometry()

        delta_time = time.time() - prev_time
        prev_time = time.time()
        full_time += delta_time

        log['time'].append(full_time)
        log['vr'].append(vr)
        log['vl'].append(vl)
    
        log_data = str(round(full_time, 2))+' '+str(round(vr, 2))+' '+str(round(vl, 2))+'\n'
        file.write(log_data)
    
    set_linear = -0.3
    set_angular = 0.0
    robot.setMotors(set_linear, set_angular)
    while full_time <= 4:
        
        x, y, yaw, v, w, vr, vl = robot.getOdometry()

        delta_time = time.time() - prev_time
        prev_time = time.time()
        full_time += delta_time

        log['time'].append(full_time)
        log['vr'].append(vr)
        log['vl'].append(vl)
    
        log_data = str(round(full_time, 2))+' '+str(round(vr, 2))+' '+str(round(vl, 2))+'\n'
        file.write(log_data)

    set_linear = 0.3
    set_angular = 0.0
    robot.setMotors(set_linear, set_angular)
    while full_time <= 6:
        
        x, y, yaw, v, w, vr, vl = robot.getOdometry()

        delta_time = time.time() - prev_time
        prev_time = time.time()
        full_time += delta_time

        log['time'].append(full_time)
        log['vr'].append(vr)
        log['vl'].append(vl)
    
        log_data = str(round(full_time, 2))+' '+str(round(vr, 2))+' '+str(round(vl, 2))+'\n'
        file.write(log_data)

    robot.setMotors(0, 0)
    robot.stop()
    file.close()

    print("max speed left: {0}", max(log['vl']))
    print("max speed right: {0}", min(log['vr']))
    
    plt.plot(log['time'], log['vr'],'-b', label='right vel[m/s]')
    plt.plot(log['time'], log['vl'],'-r', label='left vel[m/s]')
    plt.legend()
    plt.xlabel('time [sec]')
    plt.ylabel('v [m/s]')
    plt.show()

    print('Done!')