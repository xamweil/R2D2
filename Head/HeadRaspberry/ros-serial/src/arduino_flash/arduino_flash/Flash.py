import os
import subprocess
class Flash:
    def __init__(self):
        self.portUno = "/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_758343531313513150C1-if00"
        

    def flash(self, name: str):        
        if name=="Uno":
            return self.flash_uno(self.portUno)
        elif name=="Xiao8x8":
            return False, "Xiao8x8 flashing not implemented yet"
        elif name=="Xiao4x12":     
            return False, "Xiao4x12 flashing not implemented yet"
        else:
            return False, f"Unknown device: {name}"

    def flash_uno(self, port_id: str):
        """
        Flash Arduino Uno
        """
        fqbn = "arduino:avr:uno"
        dir = os.path.dirname(__file__) 
        dir = os.path.abspath(os.path.join(dir, "..", "..", "..", "..", "..", "HeadUno"))
        
        if not os.path.isdir(dir):
            return False, f"Sketch directory not found: {dir}"

        compile_cmd = [
            "arduino-cli", "compile",
            "--fqbn", fqbn,
            dir
        ]

        proc = subprocess.run(compile_cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            return False, f"Compile failed:\n{proc.stdout}{proc.stderr}"

        flash_cmd = [
            "arduino-cli", "upload",
            "-p", port_id,
            "--fqbn", fqbn,
            dir
        ]

        proc = subprocess.run(flash_cmd, capture_output=True, text=True)
        if proc.returncode != 0:
            return False, f"Upload failed:\n{proc.stdout}{proc.stderr}"

        return True, "Arduino Uno flashed successfully"
            
if __name__ == "__main__":
    flash = Flash()
    success, message = flash.flash("Uno",)
    print(message)
    if not success:
        print("Flashing failed")
    else:
        print("Flashing succeeded")
