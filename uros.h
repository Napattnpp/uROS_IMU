/****************************************************************
 //BP trying to combine imu (icm20948) with microros on ROS2
 //Testing with MKR zero arduino board
 ***************************************************************/
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float64.h>
#include <sensor_msgs/msg/imu.h>
#include <rmw_microros/time_sync.h>
#include <rosidl_runtime_c/string_functions.h>

#define LED_PIN 13

class uROS {
  private:
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;

    rcl_publisher_t yaw_publisher;
    rcl_publisher_t imu_publisher;
    std_msgs__msg__Float64 yaw_msg;
    sensor_msgs__msg__Imu imu_msg;
    unsigned long last_time_sync_ms = 0;

    void error_loop();
    void rc_check(rcl_ret_t temp_rc);
    void rc_soft_check(rcl_ret_t temp_rc);

  public:
    void init();
    void publish(
      bool imu_state,
      double yaw,
      double qw, double qx, double qy, double qz,
      double gx, double gy, double gz,
      double ax, double ay, double az
    );
};

void uROS::error_loop() {
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void uROS::rc_check(rcl_ret_t temp_rc) {
  if(temp_rc != RCL_RET_OK) {
    error_loop();
  }
}

void uROS::rc_soft_check(rcl_ret_t temp_rc) {
  if(temp_rc != RCL_RET_OK);
}

void uROS::init() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  rc_check(rclc_support_init(&support, 0, NULL, &allocator));
  rc_check(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  // #> Codex: Sync MCU epoch with ROS agent clock so IMU timestamps are in ROS time.
  rc_soft_check(rmw_uros_sync_session(1000));
  last_time_sync_ms = millis();

  rc_check(
    rclc_publisher_init_default(
      &yaw_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
      "yaw_rad"
    )
  );

  rc_check(
    rclc_publisher_init_default(
      &imu_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
      "imu/data"
    )
  );

  // #> Codex: Initialize EKF-facing IMU message and static metadata.
  sensor_msgs__msg__Imu__init(&imu_msg);
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");

  // Tunable defaults for EKF input; adjust to your sensor/noise characteristics.
  imu_msg.orientation_covariance[0] = 0.02;
  imu_msg.orientation_covariance[4] = 0.02;
  imu_msg.orientation_covariance[8] = 0.02;
  imu_msg.angular_velocity_covariance[0] = 0.01;
  imu_msg.angular_velocity_covariance[4] = 0.01;
  imu_msg.angular_velocity_covariance[8] = 0.01;
  imu_msg.linear_acceleration_covariance[0] = 0.1;
  imu_msg.linear_acceleration_covariance[4] = 0.1;
  imu_msg.linear_acceleration_covariance[8] = 0.1;
}

void uROS::publish(
  bool imu_state,
  double yaw,
  double qw, double qx, double qy, double qz,
  double gx, double gy, double gz,
  double ax, double ay, double az
) {
  if (imu_state) {
    // #> Codex: Periodically resync epoch offset to reduce timestamp drift.
    unsigned long now_ms = millis();
    if ((now_ms - last_time_sync_ms) > 60000UL) {
      rc_soft_check(rmw_uros_sync_session(100));
      last_time_sync_ms = now_ms;
    }

    // #> Codex: Publish raw yaw angle in radians for consumers that need scalar yaw.
    yaw_msg.data = yaw;
    rc_soft_check(rcl_publish(&yaw_publisher, &yaw_msg, NULL));

    // #> Codex: Fill and publish full sensor_msgs/Imu for EKF fusion.
    int64_t now_ns = rmw_uros_epoch_nanos();
    if (now_ns <= 0) {
      // Fallback if not synchronized yet.
      now_ns = (int64_t)now_ms * 1000000LL;
    }
    imu_msg.header.stamp.sec = (int32_t)(now_ns / 1000000000LL);
    imu_msg.header.stamp.nanosec = (uint32_t)(now_ns % 1000000000LL);

    imu_msg.orientation.w = qw;
    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;

    imu_msg.angular_velocity.x = gx;
    imu_msg.angular_velocity.y = gy;
    imu_msg.angular_velocity.z = gz;

    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;

    rc_soft_check(rcl_publish(&imu_publisher, &imu_msg, NULL));
  }
}
