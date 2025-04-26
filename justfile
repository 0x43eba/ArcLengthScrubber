default:
    just -l

deploy:
    arduino-cli compile --fqbn arduino:avr:uno tilt-sensor
    arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno tilt-sensor

build:
    gcc -Wall -O2 -lm -lasound clisten/sensor_tone.c -o clisten/sensor_tone

run: build
    ./clisten/sensor_tone

clean:
    rm -f clisten/sensor_tone