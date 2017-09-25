import serial
import time

ser = serial.Serial(port='COM14', baudrate=115200)

if __name__ == '__main__':
    checksum = 0
    write_byte = [0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0a]
    print(write_byte)
    while True:
        first_time = time.time()
        ser.write(write_byte)
        last_time = time.time()
        checksum += 1
        time.sleep(0.1)
        print(ser.read())
