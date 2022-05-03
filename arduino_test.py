import serial
import time
import struct

def write(arduino, x):
    arduino.write(str(x).encode())

def read(arduino):
    data = arduino.readline()
    try:
        data = data.decode()
        data = data.strip()
        data = data.split()
        if len(data) == 3:
            data = list(map(float,data))
            return data
    except:
        return None

if __name__ == "__main__":

    ard = serial.Serial()
    ard.port = "COM3"
    ard.baudrate = 115200
    ard.timeout = 1
    #ard.setDTR(False)
    #ard.setRTS(False)
    ard.open()

    # ard.flushInput()
    # ard.flushOutput()
    # ard.flush()
    time.sleep(2)
    while True:
        #write(ard, '2')
        print('write')
        x = int(input())
        write(ard,x)
        ard.write('1'.encode())
        time.sleep(1)
        # if x == '1':
        #     write(ard, 1)
        #     ard.flush()
        # # line = read(ard)
        # #line = ard.readline()
        #
        if x == '2':
            write(ard, 0)
            ard.flush()

