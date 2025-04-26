## Arc Length Scrubber

This is a small combination of Arduino code, and Python code that allows you to use the (X,Y) displacement of an accelerometer across a plain
to control a musical track.

### tilt-sensor

This is the code itself. We transmit the tilt sensor data as a compact buffer over the serial port. This data is transmitted in a six-byte packet.

### listener

This is the "client" for this setup. We're using this to pick up and manipulate the code from the embedded system.

![image](https://github.com/user-attachments/assets/0d9b4b30-a982-4bd9-952a-2960bdac18b9)
