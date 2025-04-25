default:
    just -l

deploy:
    arduino-cli compile --fqbn arduino:avr:uno
    arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno

listen:
    cd listener && uv run main.py
