#include <Wire.h>
#include <SparkFun_MMA8452Q.h>

MMA8452Q accel;

#pragma pack(push, 1)
typedef struct {
  int16_t X;
  int16_t Y;
  int16_t Z
} SensorDataOutput;
#pragma pack(pop)

void send32(uint32_t val) {
  Serial.write((val >> 24) & 0xFF);
  Serial.write((val >> 16) & 0xFF);
  Serial.write((val >> 8) & 0xFF);
  Serial.write(val & 0xFF);
}

void setup() {
  Serial.begin(38400);
  Wire.begin();

  if (!accel.begin()) {
    send32(0xDEADBEEF);
    while (1);
  }

  send32(0xFEEDFACE);
}

void loop() {
  accel.read();

  int16_t x = accel.cx * 1000;
  int16_t y = accel.cy * 1000;
  int16_t z = accel.cz * 1000;

  SensorDataOutput data = {x, y, z};
  Serial.write(0xAA);
  uint8_t buffer[sizeof(data)];
  memcpy(buffer, &data, sizeof(data));
  Serial.write(buffer, sizeof(data));
  delay(10);
}
