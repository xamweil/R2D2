import struct
from serial_comm.SerialProcessor import SerialProcessor
class Nob:
    def __init__(self, serial_processor: SerialProcessor, address):
        self.CID = address
        self.ser = serial_processor
        
    def set_posX(self, angle_x, angle_y):
        payload = struct.pack('>hh', angle_x, angle_y)
        response = self.ser.send_packet(self.CID, 0x01, payload)
        return response
    
    def set_posX(self, angle):
        payload = struct.pack('>h', angle)
        response = self.self.ser.send_packet(self.CID, 0x02, payload)
        return response
    
    def set_posY(self, angle):
        payload = struct.pack('>h', angle)
        response = self.ser.send_packet(self.CID, 0x03, payload)
        return response
    
    def get_pos(self):
        response = self.ser.send_packet(self.CID, 0x04)
        return response
    def run_circle(self):
        response = self.ser.send_packet(CID, 0x05, timeout=3)
        return response
    
        
        
    
    
        
        
        
        
        