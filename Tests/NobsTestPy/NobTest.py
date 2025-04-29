import serial
import time
import struct

# --- Settings ---
port = 'COM6'    # <<< Replace with your Arduino COM port
baud = 57600
timeout = 1      # seconds

# --- Setup serial ---
ser = serial.Serial(port, baud, timeout=timeout)
time.sleep(2)  # Wait for Arduino to reset

# --- Build a packet to move nob1 ---
# Example: Move nob1 X to 45 degrees

SOF = 0xAA
CID = 0x12   # Class ID for nob1
FID = 0x02   # Function ID for setPosX
angle = 3    # degrees

# Payload: 2 bytes uint16_t (big endian)
payload = struct.pack('>h', angle)  # 2 bytes: high, low
LEN = 1 + 1 + len(payload) + 1  # CID + FID + PAYLOAD + CRC
frame = bytearray()
frame.append(SOF)
frame.append(LEN)   # LEN without SOF+LEN itself
frame.append(CID)
frame.append(FID)
frame.extend(payload)

# Compute CRC (XOR over all bytes except CRC itself)
crc = SOF ^ frame[1]
for b in frame[2:]:
    crc ^= b
frame.append(crc)

# --- Send the packet ---
ser.write(frame)
print(f"Sent frame: {[hex(b) for b in frame]}")

# --- Read response ---
response = ser.read()
if response:
    print(f"Arduino responded with: {hex(response[0])}")
else:
    print("No response!")

ser.close()
