//#include <arduino.h>
//#include <arduino_freertos.h>
//#include "TeensyThreads.h"
#include <micro_ros_arduino.h>
#include <stdio.h>  

// FreeRTOS
#include "arduino_freertos.h"
//#include <FreeRTOS_TEENSY4.h>
//#include <Arduino_FreeRTOS.h>
//#include <freertos/FreeRTOS.h>

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
//#define LED_PIN 13
// Declare a semaphore handle.
//SemaphoreHandle_t sem;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void debug_blink(int delay_time, int countlim){
  for(int i=0; i<countlim; i++){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(delay_time);
  }
}

void setup() {
  ros_setup();
  // crate tasks of FreeRTOS
  //portBASE_TYPE ros_task;
  //xTaskCreatePinnedToCore(ros_spin_task, "ros_task", 
  //                2*1024, NULL, 4, NULL, 1);
  xTaskCreate(ros_spin_task, "ros_task", configMINIMAL_STACK_SIZE + 1000, NULL, configMAX_PRIORITIES - 1, NULL); // tskIDLE_PRIORITY+1


  // start FreeRTOS
  vTaskStartScheduler();  // pass
  //while(1);
}

void loop() {
  //delay(100);
}

void ros_setup(){
  set_microros_transports();
  
  pinMode(LED_PIN, arduino::OUTPUT);
  digitalWrite(LED_PIN, arduino::HIGH);  
  //threads.delay(150)
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "FreeRTOS_uros_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "/freertos_uros/sample/talker"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&pub_executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&pub_executor, &timer));

  debug_blink(500, 10);

  msg.data = 0;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // set msg contents
    //msg.data++;
    /*String s = "Hello FreeRTOS uros for Teensy";
    char strBuf[30];
    s.toCharArray(strBuf, 30);
    msg.data.size = s.length();
    msg.data.data = strBuf;
    */
    //debug_blink(2000, 10);
    msg.data++;

    // publish
    RCCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

// node executor
static void ros_spin_task(void *p){
  debug_blink(1000, 10);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1){
    //debug_blink(3000, 10);
    RCCHECK(rclc_executor_spin(&pub_executor));
    vTaskDelayUntil(&xLastWakeTime, 10);
    //vTaskDelay((200L*configTICK_RATE_HZ) / 1000L);
    //RCSOFTCHECK(rclc_executor_spin_some(&pub_executor, RCL_MS_TO_NS(5)));
    //RCSOFTCHECK(rclc_executor_spin_some(&sub_executor, RCL_MS_TO_NS(5)));
  }
}