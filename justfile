default:
    just -l

deploy:
    arduino-cli compile --fqbn arduino:avr:uno tilt-sensor
    arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno tilt-sensor

listen:
    cd listener && uv run main.py
