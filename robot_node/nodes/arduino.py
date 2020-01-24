import serial
import time

#Arduino protocol
set_command = 'v'
print_command = 'd'
start_connect = 's'

class protocol():
    def __init__(self, port, rate):
        self.connect = self.openconnect(port, rate)

    def check_connect(self, connect):
        c = connect.read(1).decode()
        if c != 'c':
            print('Connect close..')
            self.stop()

    def openconnect(self, port, rate):
        connect = serial.Serial(port, rate)
        time.sleep(1)
        while not connect.is_open:
            self.openconnect(port, rate)
        is_connected = False
        while not is_connected:
            print("Waiting for arduino...")
            connect.write(start_connect.encode())
            connect_flag = connect.read(1).decode()
            self.check_connect(connect)
            if not connect_flag:
                time.sleep(0.1)
                continue
            if connect_flag == 'r':
                is_connected = True
                print('Connected!')
                
        return connect

    def readline(self, connect):
        line = bytearray()
        if connect.inWaiting:
            while True:
                c = connect.read(1)
                if "c" in c:
                    break
                else:
                    line+=c
        return bytes(line)

    def arduino_stop(self):
        self.setMotors(0,0)
        self.connect.close()

    def stop(self):
        self.arduino_stop()

    def getOdometry(self):
        self.connect.write(print_command.encode())
        data = self.readline(self.connect).decode() 
        data = data.split(';')
        x = float(data[0])
        y = float(data[1])
        yaw = float(data[2])
        v = float(data[3])
        w = float(data[4])
        vr = float(data[5])
        vl = float(data[6])

        return x, y, yaw, v, w, vr, vl
 
    def setMotors(self, v, w):
        self.send(v, w)

    def send(self, lvel, avel):
        lvel_sign = ''
        avel_sign = ''
        if (lvel >=0):
            lvel_sign = '+'
        else:
            lvel_sign = '-'
        if (avel >=0):
            avel_sign = '+'
        else:
            avel_sign = '-'
        send_data = set_command + lvel_sign + str(round(abs(lvel),2)) + ' ' + avel_sign + str(round(abs(avel),2)) + "\n"
        self.connect.write(send_data.encode())
        self.check_connect(self.connect)
    
if __name__ == '__main__':
    from arduino import protocol
    connect = protocol('/dev/ttyACM1',57600)
    
    x = 0
    while x < 20:
        connect.send(0.2, 0.0)
        odometry = connect.getOdometry()
        
        print(odometry)
    connect.stop()