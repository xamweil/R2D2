import struct

class LedMatrix:
    def __init__(self, address, serial_processor):
        
        self.CID = address
        self.ser = serial_processor

    def random_lights(self):
        response = self.ser.send_packet(self.CID, 0x01)
        return response

    def display_off(self):
        response = self.ser.send_packet(self.CID, 0x02)
        return response
    
    def display_on(self):
        response = self.ser.send_packet(self.CID, 0x03)
        return response

    def set_leds(self, payload):
        response = self.ser.send_packet(self.CID, 0x04, payload)
        return response
    

    