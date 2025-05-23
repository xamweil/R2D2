from Lid import Lid
from Nob import Nob
from SerialProcessor import SerialProcessor
import socket
class CommandServer:
    def __init__(self, host='0.0.0.0', port='12345'):
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        
        self.ser = Serial.SerialProcessor('COM6')
        
        
        self.devices = {
            "lid1_1": Lid(self.ser, 0x01),
            "lid1_2": Lid(self.ser, 0x02),
            "lid1_3": Lid(self.ser, 0x03),
            "lid1_4": Lid(self.ser, 0x04),
            "lid1_5": Lid(self.ser, 0x05),
            "lid2_1": Lid(self.ser, 0x06),
            "lid2_2": Lid(self.ser, 0x07),
            "lid2_3": Lid(self.ser, 0x08),
            "lid2_4": Lid(self.ser, 0x09),
            "lid2_5": Lid(self.ser, 0x0A),
            "nob1": Nob(self.ser, 0x10),
            "nob2": Nob(self.ser, 0x11),
            "nob3": Nob(self.ser, 0x12),
        }
        
    def start(self):
        while True:
            client, addr = self.server_socket.accept()
            
            with client:
                while True:
                    data = client.recv(1024)
                    if not data:
                        break
                    try:
                        command = json.loads(data.decode())
                        result = self.execute_command(command)
                        response = json.dumps({"status": "ok", "result": result})
                        
                    except Exception as e:
                        responds = json.dumps({"status": "error", "message:": str(e)})
                    client.send(response.encode())
    def execute_command(self, cmd):
        target_name = cmd["target"]
        action = cmd["action"]
        args = cmd.get("args", [])
        kwargs = cmd.get("kwargs", [])
        
        target = self.devices.get(target_name)
        if not target:
            raise ValueError(f"Device '{target_name}' not found")

        method = getattr(target, action, None)
        if not method:
            raise ValueError(f"Action '{action}' not found on {target_name}")

        result = method(*args, **kwargs)
        return result  # Will be sent back to the client
                        
        
        
    

            
    