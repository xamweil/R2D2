# Six‑axis sample from an MPU‑6500 + device‑local timestamp (ms)

int16[3] accel     # ax, ay, az  (raw counts)
int16[3] gyro      # gx, gy, gz  (raw counts)
uint32   ts_ms     