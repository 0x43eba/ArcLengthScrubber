import serial
import logging
import struct

MAGIC = b'\xFE\xED\xFA\xCE'  # Big-endian FEEDFACE
START_BYTE = b'\xAA'
HEADER_LEN = 4
SENSOR_STRUCT_LEN = 6  # 3 x int16_t

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger(__name__)

try:
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    logger.info("Listening on /dev/ttyACM0 at 9600 baud...")

    buffer = bytearray()
    synced = False

    while True:
        byte = ser.read(1)
        if not byte:
            continue

        if not synced:
            buffer += byte
            if len(buffer) > HEADER_LEN:
                buffer = buffer[-HEADER_LEN:]  # keep last 4 bytes
            if buffer == MAGIC:
                synced = True
                logger.info("Detected 0xFEEDFACE — now reading data packets...")
        else:
            if byte != START_BYTE:
                continue
            payload = ser.read(SENSOR_STRUCT_LEN)
            if len(payload) != SENSOR_STRUCT_LEN:
                continue
            x, y, z = struct.unpack('<hhh', payload)  # little-endian 3x int16_t
            logger.info(f"X: {x}, Y: {y}, Z: {z}")

except serial.SerialException as e:
    logger.error(f"Serial error: {e}")

except KeyboardInterrupt:
    logger.info("Stopped by user.")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        logger.info("Serial port closed.")
