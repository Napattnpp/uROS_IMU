#define SERIAL_PORT Serial

#include "uros.h"
// #include "motor_encoder.h"
#include "imu.h"

uROS uros;
IMU imu;
// MotorEncoder motor_encoder;

void setup() {
  SERIAL_PORT.begin(115200);

  uros.init();
  // motor_encoder.init();
  imu.init();
}

void loop() {
  // motor_encoder.run();
  imu.run();

  uros.publish(
    imu.publish_state,
    imu.yaw,
    imu.qw, imu.qx, imu.qy, imu.qz,
    imu.gx, imu.gy, imu.gz,
    imu.ax, imu.ay, imu.az
  );
}
