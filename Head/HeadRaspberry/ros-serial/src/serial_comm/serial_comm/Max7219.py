
class Max7219:
    def __init__(self, serial_processor, address = 0x01):
        self.CID = address
        self.ser = serial_processor

    def random_lights(self, payload=[]):
        response = self.ser.send_packet(self.CID, 0x01, payload)
        return response
    def display_on(self):
        response = self.ser.send_packet(self.CID, 0x02)
        return response
    def display_off(self):
        response = self.ser.send_packet(self.CID, 0x03)
        return response
    def set_leds(self, payload):
        response = self.ser.send_packet(self.CID, 0x04, payload)
        return response
    def set_single_led(self):
        response = self.ser.send_packet(self.CID, 0x05, payload)
        return response