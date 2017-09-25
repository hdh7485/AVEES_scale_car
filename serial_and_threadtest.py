import serial
import threading
from time import sleep

serial_port = serial.Serial()

send_data = [0x02, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x03]
#send_data = [0x02, 0x02, 0x04, 0x03] 

def read():
    while True:
        input_data = []
        data = serial_port.read()
#        print(data)
        if int.from_bytes(data, byteorder='little') == 0x02:
            while True:
                data = serial_port.read();
                if int.from_bytes(data, byteorder='little') == 0x03: break
                input_data.append(data)
            print(input_data)

def main():
    checksum = 0
    serial_port.baudrate = 115200 
    serial_port.port = 'COM14'
    serial_port.timeout = None
    serial_port.open()
    t1 = threading.Thread(target=read, args=())
    t1.start()
    while True:
        if checksum > 255 : checksum = 0
        send_data[10] = checksum
        serial_port.write(send_data)
        checksum += 1
    serial_port.close()

if __name__ == "__main__":
    main()
