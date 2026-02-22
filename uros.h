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

#define LED_PIN 13

class uROS {
  private:
    rclc_support_t support;
    rcl_allocator_t allocator;
    rcl_node_t node;

    rcl_publisher_t heading_publisher;
    std_msgs__msg__Float64 heading_msg;

    void error_loop();
    void rc_check(rcl_ret_t temp_rc);
    void rc_soft_check(rcl_ret_t temp_rc);

  public:
    void init();
    void publish(bool imu_state, double yaw);
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

  rc_check(
    rclc_publisher_init_default(
      &heading_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
      "heading_deg"
    )
  );
}

void uROS::publish(bool imu_state, double yaw) {
  if (imu_state) {
    double heading_deg = yaw * 180.0 / PI;
    if (heading_deg < 0.0) {
      heading_deg += 360.0;
    }
    heading_msg.data = heading_deg;
    rc_soft_check(rcl_publish(&heading_publisher, &heading_msg, NULL));
  }
}
