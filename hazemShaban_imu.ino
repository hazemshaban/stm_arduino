#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
#define OUTPUT_READABLE_ACCELGYRO


void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  Serial1.begin(115200);

  // initialize device
  Serial1.println("Initializing I2C devices...");
  accelgyro.initialize();
  Serial1.println("Testing device connections...");
  Serial1.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  // or accelgyro.getAcceleration(&ax, &ay, &az);
#ifdef OUTPUT_READABLE_ACCEL
  Serial1.print("a/g:\t");
  Serial1.print(ax); Serial1.print("\t");
  Serial1.print(ay); Serial1.print("\t");
  Serial1.print(az); Serial1.print("\t");
#endif
  pitch = 180 * atan (ax / sqrt(ay * ay + az * az)) / M_PI;
  roll = 180 * atan (ay / sqrt(ax * ax + az * az)) / M_PI;
  yaw = 180 * atan (az / sqrt(ax * ax + az * az)) / M_PI;
  delay(100);
  Serial1.print(pitch); Serial1.print("\t");
  Serial1.print(roll); Serial1.print("\t");
  Serial1.print(yaw); Serial1.print("\t");


}
