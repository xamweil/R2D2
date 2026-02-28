import struct

class CameraTiltActuator:
    def __init__(self, serial_processor, address):
        self.CID = address
        self.ser = serial_processor

    def default_pos(self):
        response = self.ser.send_packet(self.CID, 0x01)
        return response

    def low_pos(self):
        return self.ser.send_packet(self.CID, 0x02)

    def set_default_pos(self, payload):
        return self.ser.send_packet(self.CID, 0x03, payload)
    
    def set_low_pos(self, payload):
        return self.ser.sendpacket(self.CID, 0x04, payload)

    def set_pos(self, payload):
        return self.ser.send_packet(self.CID, 0x05, payload)

    def get_pos(self):
        return self.ser.send_packet(self.CID, 0x06)