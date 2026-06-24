e-puck Monitor
==============
EPFL e-puck Mini Robot - PC Monitoring & Control Application

Requirements
------------
- Windows 10/11
- Bluetooth serial port (or USB serial) paired with e-puck robot
- e-puck robot running BTcom firmware

Usage
-----
1. Ensure your e-puck robot is powered on and paired via Bluetooth (or connected via USB serial).
2. Launch "e-puck Monitor.exe"
3. Select the COM port from the dropdown (default: COM6)
4. Click "Connect"
5. Use the controls to:
   - View camera images (single shot or continuous)
   - Read proximity sensors (8 IR sensors around the robot)
   - Control LEDs (8 ring LEDs + body LED + front LED)
   - Read accelerometer orientation and inclination
   - Read microphone levels
   - Drive the robot via the touch-pad
   - Test all actuators

Data Storage
------------
All runtime data is stored in the "epuck_data" folder next to the application.
No data is written to C: drive.

Troubleshooting
---------------
- If connection fails, verify the COM port number in Windows Device Manager
- The e-puck robot must be running the BTcom firmware
- Default serial settings: 115200 baud, 8 data bits, no parity, 1 stop bit
