import serial
import time

def write(arduino, x):
    arduino.write(x.encode())

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

    ard = serial.Serial('COM3', 115200, timeout=1)
    ard.flushInput()
    ard.flushOutput()
    time.sleep(1)

    while True:
        #write(ard, '2')
        #ard.write(bytes(1, 'utf-8'))
        line = read(ard)
        #line = ard.readline()
        print(line)
