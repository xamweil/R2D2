import struct
class Lid:
    def __init__(self, serial_processor, address):
        self.CID = address
        self.ser = serial_processor
        
    def open(self):
        response = self.ser.send_packet(self.CID, 0x01)
        return response
    
    def close(self):
        response = self.ser.send_packet(self.CID, 0x02)
        return response
    
    def set_open_pos(self, angle):
        payload = struct.pack('>h', angle)
        response = self.ser.send_packet(self.CID, 0x03, payload)
        return response
    
    def set_close_pos(self, angle):
        payload = struct.pack('>h', angle)
        response = self.ser.send_packet(self.CID, 0x04, payload)
        return response
        
    def get_positions(self):
        response = self.ser.send_packet(self.CID, 0x05)
        return response
    
    
    
                
        
        
        
        
        
        