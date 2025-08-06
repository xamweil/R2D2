
class XiaoESP32C3:
    def __init__(self, transport):
        self.transport = transport


    def lock_ankle(self):
        resp = self.transport.send_frame(0x03, 0x01, timeout=2)
        return resp

    def unlock_ankle(self):
        resp = self.transport.send_frame(0x03, 0x02, timeout=2)
        return resp
        
    def move_ankle(self, steps):
        payload = list(struct.pack('>h', steps))  # Convert steps to bytes
        resp = self.transport.send_frame(0x03, 0x04, payload=payload, timeout=2)

    def get_temperature(self):
        resp = self.transport.send_frame(0x01, 0x01)
        return unpack_temperature(resp)

    def read_buffer(self):
        resp = self.transport.send_frame(0x01, 0x02)
        return self.unpack_sensor_data(resp)

    def unpack_sensor_data(self, frame):
        if len(frame) != 18:                      # 2 (CID+FID) + 16 payload
            raise ValueError(f"expected 18 bytes, got {len(frame)}")

        payload = frame[2:]                       # drop CID, FID

        ax, ay, az, gx, gy, gz, ts = struct.unpack('<hhhhhhI', payload)
        return {
            'accel': (ax, ay, az),
            'gyro' : (gx, gy, gz),
            'ts_ms': ts
        }

    def unpack_temperature(frame):
        if len(frame) != 6:
            raise ValueError(f"Expected 6 bytes (SOF+LEN+CID+FID+2), got {len(frame)}")
        
        temp_bytes = frame[4:]
        temp = struct.unpack("<h", temp_bytes)[0]
        return temp
    
