"""Record the robot's Serial output to a file, with the time on each line, while it is plugged into this PC.
Finds the Arduino GIGA on any COM port (it comes back on a different one after an upload) and reconnects
by itself when the cable is unplugged and plugged in again. Stop it with Ctrl+C. Don't open the Serial
monitor at the same time: only one program can use the port.

  python tools/serial_log.py robot_log.txt            # record for up to 60 minutes
  python tools/serial_log.py robot_log.txt 120        # up to 120 minutes

Needs pyserial (PlatformIO's Python has it: %USERPROFILE%\\.platformio\\penv\\Scripts\\python.exe).
tools/robot_log.py reads the result.
"""
import sys, time
import serial, serial.tools.list_ports

out = open(sys.argv[1], "a", encoding="utf-8", buffering=1)
end = time.time() + 60 * (float(sys.argv[2]) if len(sys.argv) > 2 else 60)
quiet = 0
while time.time() < end:
    ports = [p.device for p in serial.tools.list_ports.comports() if p.vid == 0x2341]  # Arduino
    if not ports:
        if quiet % 30 == 0:
            print(f"{time.strftime('%H:%M:%S')} robot not plugged in"); out.write(f"--- {time.strftime('%H:%M:%S')} robot not plugged in\n")
        quiet += 1
        time.sleep(2)
        continue
    try:
        with serial.Serial(ports[0], 115200, timeout=1) as s:
            print(f"{time.strftime('%H:%M:%S')} recording from {ports[0]}"); out.write(f"--- connected on {ports[0]} at {time.strftime('%H:%M:%S')}\n")
            quiet = 0
            while time.time() < end:
                line = s.readline()
                if line:
                    out.write(time.strftime("%H:%M:%S ") + line.decode("utf-8", "replace").rstrip() + "\n")
    except Exception as e:
        print(f"{time.strftime('%H:%M:%S')} disconnected"); out.write(f"--- {time.strftime('%H:%M:%S')} disconnected ({type(e).__name__})\n")
        time.sleep(2)
