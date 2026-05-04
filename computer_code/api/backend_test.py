import serial
import json
import time

# Change this to your sender ESP32's port
ser = serial.Serial('/dev/cu.usbserial-XXXXX', 1000000)

# Send a test position
msg = '0' + json.dumps({"pos": [0, 0, 1, 0], "vel": [0, 0, 0]})
ser.write(msg.encode('utf-8'))
print("Sent:", msg)

time.sleep(1)

# Send arm command
msg = '0' + json.dumps({"armed": True})
ser.write(msg.encode('utf-8'))
print("Sent:", msg)

ser.close()