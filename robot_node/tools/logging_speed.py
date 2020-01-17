import time
import matplotlib.pyplot as plt
from arduino import protocol


if __name__ == '__main__':
    file = open('log_vel_nopid.txt', 'w')

    robot = protocol('/dev/ttyACM0', 57600)
    prev_time = 0
    full_time = 0
    log = {'time':[], 'vr':[], 'vl':[]}

    set_speed = 0.2
    robot.setMotors(round(set_speed,1), round(0,1))
    while full_time <= 5:
        
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
    
    plt.plot(log['time'], log['vr'],'-b')
    plt.plot(log['time'], log['vl'],'-r')
    plt.show()

    print('Done!')