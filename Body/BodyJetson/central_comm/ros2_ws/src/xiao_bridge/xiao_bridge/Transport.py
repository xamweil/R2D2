"""
 * ┌──────┬──────┬────────┬────────────────────────────────────────────┐
 * │ Byte │ Name │ Size   │ Description                                │
 * ├──────┼──────┼────────┼────────────────────────────────────────────┤
 * │ 0    │ SOF  │ 1 byte │ 0xAA start-of-frame marker                 │
 * │ 1    │ LEN  │ 1 byte │ length of (ClassID + FuncID + Payload + CRC) │
 * │ 2    │ CID  │ 1 byte │ Class ID (object type)                     │
 * │ 3    │ FID  │ 1 byte │ Func ID (method within that class)         │
 * │ 4…N  │ DATA │ LEN-3  │ Payload bytes (parameters, 0…LEN-3 bytes)  │ 
 * └──────┴──────┴────────┴────────────────────────────────────────────┘
 *
 * Total frame size = 2 (SOF+LEN) + LEN
 *
 * Example: Set lid #3 open to 90° (uint16_t payload)
 *   SOF = 0xAA
 *   LEN = 1+1+2+1 = 5
 *   CID = 0x01 (LID)
 *   FID = 0x03 (setOpenPos)
 *   DATA = 0x00 0x5A
 *
 * Return sig:
 *   0x00 = OK
 *   0x01 = bad checksum
 *   0x02 = unknown class
 *   0x03 = unknown function
 *   0x04 = bad payload
 *   0x05 = sanity check failed. Either packet is too long or len<3 (min packet size)
 *
 *   0x10-0x1F => Lid Specific errors
 *   0x20-0x2F => Nob Specific errors
 """
 # ----- CID assignments -------------------------------------------------
CID_SENSOR_LEG  = 0x01   # IMU in the tibia
CID_SENSOR_FOOT = 0x02   # IMU in the foot
CID_MOTOR       = 0x03
CID_TASTER1     = 0x04   # unsolicited press frame
CID_TASTER2     = 0x05
# ----------------------------------------------------------------------
import struct
import queue
import socket
import threading
import time

class Transport:
    def __init__(self, ip, port, timeout=5.0):
        self.ip = ip
        self.port = port
        self.timeout = timeout
        self.sock = socket.create_connection((ip, port), timeout=timeout)
        self._lock = threading.Lock()
        self.SOF = 0xAA

        self._sample_cb = None             # user callback
        self._taster_cb = None      # user callback for taster 1
        self._reply_lock = threading.Lock()
        self._reply_event = threading.Event()
        self._reply_frame = None
        self._running = True
        
        # start the reader thread
        self._rx_thread = threading.Thread(target=self._reader, daemon=True)
        self._rx_thread.start()

    def set_sample_callback(self, fn):      # fn(cid:int, payload:bytes)
        self._sample_cb = fn
    def set_taster_callback(self, fn):  # fn(cid:int, fid:int)
        self._taster_cb = fn

    def connect(self):
        while self._running:
            try:
                print(f"[Transport] Connecting to {self.ip}:{self.port} …")
                self.sock = socket.create_connection((self.ip, self.port), timeout=self.timeout)
                print("[Transport] Connected to ESP32")
                return
            except Exception as e:
                print(f"[Transport] Connection failed: {e}")
                time.sleep(2)  # Wait before retry

    
    def _reader(self):
        """Background thread: read 1 frame at a time and dispatch."""
        while self._running:
            try:
                # normal read loop on the current socket
                header = self._rec_exact(2)          # SOF + LEN
                if header[0] != self.SOF:
                    continue                         # resync

                length  = header[1]
                body    = self._rec_exact(length)
                cid, fid = body[0], body[1]
                payload = body[2:]

                # Sensor samples 
                if cid in (CID_SENSOR_LEG, CID_SENSOR_FOOT) and fid == 0x02:
                    if self._sample_cb:
                        self._sample_cb(cid, payload)   # 16‑byte payload
                    continue

                # Taster presses
                if cid == 0x04 and self._taster_cb:
                    self._taster_cb(cid, fid)
                    continue

                # Command replies
                with self._reply_lock:
                    self._reply_frame = bytes(header + body)
                    self._reply_event.set()

            except Exception as e:
                print(f"[Transport reader] Disconnected: {e}")
                try:
                    self.sock.close()
                except Exception:
                    pass

                print("[Transport reader] Attempting reconnect...")
                # clear any waiter from the old connection
                with self._reply_lock:
                    self._reply_frame = None
                    self._reply_event.clear()
                self.connect()
                continue

    def send_frame(self, CID, FID, payload=[], timeout=1):
        length = len(payload)+2 # CID + FID + PAYLOAD

        frame = bytearray([self.SOF, length, CID, FID])

        if payload:
            frame.extend(payload)

        with self._lock:
            self.sock.sendall(frame)

        
        # wait until reader thread sets the event or timeout
        if not self._reply_event.wait(timeout):
            raise TimeoutError("command reply timed out")

        with self._reply_lock:
            reply = self._reply_frame
            self._reply_frame = None
            self._reply_event.clear()


         # handle response based on payload length
        data_len = reply[1]
        payload_len = data_len - 2  # remove CID, FID

        if payload_len == 1:
            return self.interprete_response(reply)

        elif payload_len == 2:
            temp = struct.unpack("<h", reply[4:])[0]
            return temp

        elif payload_len == 16:
            return reply[2:]

        else:
            raise ValueError(f"Unknown reply format: {len(reply)} bytes")

        


    def _rec_exact(self, length):
        data = bytearray()
        while len(data) < length:
            chunk = self.sock.recv(length - len(data))
            if not chunk:
                raise IOError("Socket closed")
            data.extend(chunk)
        return data

    def interprete_response(self, res):
        code = res[4] if len(res) >= 5 else 0x05
        match code:
            case 0x00:
                return "OK"
            case 0x01:
                return "Bad checksum"
            case 0x02:
                return "Unknown class"
            case 0x03:
                return "Unknown function"
            case 0x04:
                return "Bad payload"
            case 0x05:
                return "Sanity check failed (packet too long or len<3)"
            case 0x10:
                return res[2:]
            case _:
                return f"Unknown error code {res}"
