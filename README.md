## Arc Length Scrubber

This is a small combination of Arduino code, and Python code that allows you to use the (X,Y) displacement of an accelerometer across a plain
to control a musical track.

### tilt-sensor

This is the code itself. We transmit the tilt sensor data as a compact buffer over the serial port.

### listener

This is the "client" for this setup. We're using this to pick up and manipulate the code from the embedded system.
