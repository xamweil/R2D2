from SerialProcessor import SerialProcessor
import time
from Lid import Lid
from Nob import Nob
from Max7219 import Max7219
import serial

def initialize(serUno, serXiao8x8, serXiao4x12):
    devices = {
            "lid1_1": Lid(serUno, 0x01),
            "lid1_2": Lid(serUno, 0x02),
            "lid1_3": Lid(serUno, 0x03),
            "lid1_4": Lid(serUno, 0x04),
            "lid1_5": Lid(serUno, 0x05),
            "lid2_6": Lid(serUno, 0x06),
            "lid2_1": Lid(serUno, 0x07),
            "lid2_2": Lid(serUno, 0x08),
            "lid2_3": Lid(serUno, 0x09),
            "lid2_4": Lid(serUno, 0x0A),
            "lid2_5": Lid(serUno, 0x0B),
            "nob1": Nob(serUno, 0x10),
            "nob2": Nob(serUno, 0x11),
            "nob3": Nob(serUno, 0x12),
            "Max7219": Max7219(serXiao8x8, 0x01),
        }
    return devices

    


if __name__ == "__main__":
    portUno = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_758343531313513150C1-if00"
    portXiao8x8 = "/dev/serial/by-id/usb-Seeed_Seeed_XIAO_M0_3FE8661E5154315432202020FF0B191E-if00"
    portXiao4x12 = "/dev/serial/by-id/usb-Seeed_Seeed_XIAO_M0_391D567A5153593336202020FF0A1625-if00"
    serUno = SerialProcessor(portUno)
    serXiao8x8 = SerialProcessor(portXiao8x8)
    serXiao4x12 = SerialProcessor(portXiao4x12)
    devices = initialize(serUno, serXiao8x8, serXiao4x12)
    time.sleep(2)  # Allow some time for the serial port to initialize
    
    serXiao4x12.write("chase_col\n".encode('utf-8'))  
    serXiao8x8.write("random\n".encode('utf-8'))  
    """
    for device in list(devices.values())[:10]:
        print(device.open())
        time.sleep(1)
    for device in list(devices.values())[:10]:
        print(device.close())
        time.sleep(1)
    """
    