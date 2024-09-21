#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>  
#include "TeensyThreads.h"

// ros
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t pub_executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

const int LED_PIN = 13;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void debug_blink(int blinkNum, int delay_time=300){
  for(int i=0; i<blinkNum+1; i++){
    digitalWrite(LED_PIN, HIGH);
    delay(delay_time);
    digitalWrite(LED_PIN, LOW);
    delay(delay_time);
    //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    //threads.delay(delay_time);
  }
  delay(2000);
}

void setup() {
  // ros2 node setup
  ros_setup();

  // initialize semaphore
  threads.addThread(ros_spin_task);
}

void ros_setup(){
  set_microros_transports();
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  debug_blink(3);
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  debug_blink(4);
  RCCHECK(rclc_node_init_default(&node, "teensythreads_uros_node", "", &support));

  // create publisher
  debug_blink(5);
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/teenthreads_uros/sample/talker"));

  // create timer,
  debug_blink(6);
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  debug_blink(7);
  RCCHECK(rclc_executor_init(&pub_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&pub_executor, &timer));

  msg.data = 0;

  // setup success blink
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    msg.data++;
    // publish
    RCCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

void loop() {
  // data update via multithreads
  //msg.data++;
}

// node executor
static void ros_spin_task(){
  while(1){
    RCSOFTCHECK(rclc_executor_spin_some(&pub_executor, RCL_MS_TO_NS(5)));
    
    // set a ros thread
    threads.yield();
  }
}