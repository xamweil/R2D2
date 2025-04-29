frame = [
    0xAA,  # SOF
    0x05,  # LEN
    0x10,  # CID
    0x02,  # FID
    0x00,  # Payload High byte
    0x05   # Payload Low byte
]

# 1) Full wrong way (wrong: SOF included in loop)
crc_wrong = 0
for b in frame:
    crc_wrong ^= b
print(f"Wrong CRC (SOF inside loop): {hex(crc_wrong)}")

# 2) Arduino matching correct way
crc_correct = frame[0] ^ frame[1]  # SOF ⊕ LEN
for b in frame[2:]:                # XOR CID, FID, payload
    crc_correct ^= b
print(f"Correct Arduino-style CRC: {hex(crc_correct)}")