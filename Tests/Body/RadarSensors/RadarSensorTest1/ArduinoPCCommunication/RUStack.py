import serial
import serial.tools.list_ports

GYRO_RANGETABLE = (
(250,b">MPU6886.SETSCALE(GSCALE=250DPS)"),
(500,b">MPU6886.SETSCALE(GSCALE=500DPS)"),
(1000,b">MPU6886.SETSCALE(GSCALE=1000DPS)"),
(2000,b">MPU6886.SETSCALE(GSCALE=2000DPS)")
)

ACC_RANGETABLE = (
(2,b">MPU6886.SETSCALE(ASCALE=2g)"),
(4,b">MPU6886.SETSCALE(ASCALE=4g)"),
(8,b">MPU6886.SETSCALE(ASCALE=8g)"),
(16,b">MPU6886.SETSCALE(ASCALE=16g)")
)



def findport():
    ports = serial.tools.list_ports.comports()
    portfound=False
    portname=''
    for port, desc, hwid in sorted(ports):
        if 'VID:PID=10C4:EA60' in hwid: # check if serial port uses this UART
            portfound=True
            portname=port
            break
    return(portfound,portname)

def M5_response_to_float(rsp):
    try:
        rsp_string=rsp.decode('UTF-8')
        rsp_string=rsp_string.replace('(', '')
        rsp_string=rsp_string.replace(')', '')
        return float(rsp_string)
    except:
        return(-99999)


class Display():
    def __init__(self,RUStack_parent):

        self.stack=RUStack_parent
    def turn_on(self):
        self.stack.sendraw(b">Display.activate()")
    def turn_off(self):
        self.stack.sendraw(b">Display.deactivate()")

class Audio():
    def __init__(self,RUStack_parent):

        self.stack=RUStack_parent
    def beep(self):
        self.stack.sendraw(b">Audio.beep()")

class Esp32_hall:
    def __init__(self,RUStack_parent):
        self.stack=RUStack_parent
        self._value=0

    @property
    def value(self):
        return self.stack.read_esp32_hall()


class MPU6886():
    def __init__(self,RUStack_parent):

        self.stack=RUStack_parent

        self._accX=0
        self._accY=0
        self._accZ=0

        self._gyroX=0
        self._gyroY=0
        self._gyroZ=0

        self._pitch=0
        self._roll=0
        self._yaw=0


    @property
    def accX(self):
        return self.stack.read_accX()

    @property
    def accY(self):
        return self.stack.read_accY()

    @property
    def accZ(self):
        return self.stack.read_accZ()



    @property
    def gyroX(self):
        return self.stack.read_gyroX()

    @property
    def gyroY(self):
        return self.stack.read_gyroY()

    @property
    def gyroZ(self):
        return self.stack.read_gyroZ()



    @property
    def pitch(self):
        return self.stack.read_pitch()

    @property
    def roll(self):
        return self.stack.read_roll()

    @property
    def yaw(self):
        return self.stack.read_yaw()

    def set_gyro_range(self,r):
        for (range, sendstring) in GYRO_RANGETABLE:
            if range==r:
                self.stack.sendraw(sendstring)
                break

    def set_acc_range(self,r):
        for (range, sendstring) in ACC_RANGETABLE:
            if range==r:
                self.stack.sendraw(sendstring)
                break





class RUStack():

    def __init__(self):
        self.ser=serial.Serial()
        self.mpu6886=MPU6886(self)
        self.display=Display(self)
        self.audio=Audio(self)
        self.esp32_hall=Esp32_hall(self)
        self.connected=False

    def connect(self,port):

        self.ser.port=port
        self.ser.timeout=1
        self.ser.baudrate=115200
        self.ser.rts=0
        self.ser.dtr=0

        try:
            self.ser.open()
            self.ser.flush()
            self.connected=True
        except:
            self.connected=False

    def autoconnect(self):
        found,portname=findport()
        if found:
            self.ser=serial.Serial()
            self.ser.port=portname
            self.ser.timeout=1
            self.ser.baudrate=115200
            self.ser.rts=0
            self.ser.dtr=0
            try:
                self.ser.open()

                self.connected=True
            except:
                self.connected=False
        else:
            self.connected=False

    def autoconnect_linux(self):
        found,portname=findport()
        if found:
            self.ser=serial.Serial()
            self.ser.port=portname
            self.ser.timeout=1
            self.ser.baudrate=115200
            try:
                self.ser.open()

                self.connected=True
            except:
                self.connected=False
        else:
            self.connected=False

    def disconnect(self):
        if self.ser.isOpen():
            self.ser.close()
        self.connected=False

    def sendraw(self,s):
        if (self.connected):
            self.ser.write(s)
            response=self.ser.readline()
            return response
        else:
            return(None)

    # Hall sensor
    def read_esp32_hall(self):
        if (self.connected):
            self.ser.write(b">ESP32_Hall.read()")
            response=self.ser.readline()
            return M5_response_to_float(response)
        else:
            return(None)

    # accelerometers
    def read_accX(self):
        if (self.connected):
            self.ser.write(b">MPU6886.accX()")
            response=self.ser.readline()
            return M5_response_to_float(response)
        else:
            return(None)
    def read_accY(self):
        if (self.connected):
            self.ser.write(b">MPU6886.accY()")
            response=self.ser.readline()
            return M5_response_to_float(response)
        else:
            return(None)
    def read_accZ(self):
        if (self.connected):
            self.ser.write(b">MPU6886.accZ()")
            response=self.ser.readline()
            return M5_response_to_float(response)
        else:
            return(None)

    # gyros
    def read_gyroX(self):
        if (self.connected):
            self.ser.write(b">MPU6886.gyroX()")
            response=self.ser.readline()
            return M5_response_to_float(response)
        else:
            return(None)
    def read_gyroY(self):
        if (self.connected):
            self.ser.write(b">MPU6886.gyroY()")
            response=self.ser.readline()
            return M5_response_to_float(response)
        else:
            return(None)
    def read_gyroZ(self):
        if (self.connected):
            self.ser.write(b">MPU6886.gyroZ()")
            response=self.ser.readline()
            return M5_response_to_float(response)
        else:
            return(None)

    # compass
    def read_pitch(self):
        if (self.connected):
            self.ser.write(b">MPU6886.pitch()")
            response=self.ser.readline()
            return M5_response_to_float(response)
        else:
            return(None)
    def read_roll(self):
        if (self.connected):
            self.ser.write(b">MPU6886.roll()")
            response=self.ser.readline()
            return M5_response_to_float(response)
        else:
            return(None)
    def read_yaw(self):
        if (self.connected):
            self.ser.write(b">MPU6886.yaw()")
            response=self.ser.readline()
            return M5_response_to_float(response)
        else:
            return(None)

    def beep(self):
        if (self.connected):
            self.ser.write(b">Audio.Beep()")
            response=self.ser.readline()


