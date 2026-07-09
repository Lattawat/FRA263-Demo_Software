#include <Arduino.h>

#define LED_PIN 13
#define EN_CH_A 7
#define EN_CH_B 6
#define EN_RES 4096.0
#define ticks2rad(ticks) ((ticks/EN_RES)*2.0*PI)

// ── Group namespace ─────────────────────────────────────────────────────────
// Set GROUP_NUMBER per group and reflash. The node is placed under the ROS
// namespace /G<GROUP_NUMBER>, so it publishes /G<N>/encoder_raw — matching the
// namespaced encoder_reader (which subscribes to the relative "encoder_raw"
// under /G<N> from `group_number:=N`). GROUP_NUMBER here MUST match that N.
// The two-level STR/STR2 stringize turns the number's VALUE (not its name)
// into a string, then adjacent string literals concatenate at compile time.
#define GROUP_NUMBER 0
#define STR2(x) #x
#define STR(x)  STR2(x)
#define ROS_NAMESPACE "G" STR(GROUP_NUMBER)     // → "G0", "G5", …
// If a given micro-ROS build does not expand the relative topic under the node
// namespace, fall back to an absolute topic instead:
//   #define ENCODER_TOPIC "/G" STR(GROUP_NUMBER) "/encoder_raw"

// #define SERIAL_DEBUG true

#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <claude_visualizer_interface/msg/encoder_raw.h>
#include <rmw_microros/rmw_microros.h>

#include "Encoder.h"

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This script is only available for Arduino framework with serial transport.
#endif

claude_visualizer_interface__msg__EncoderRaw encoder_raw;

rclc_support_t support; //Holds initialization context shared across all ROS 2 entities (node, timers, etc.).
rcl_allocator_t allocator = rcl_get_default_allocator(); //Manages memory allocation — micro-ROS needs explicit allocator control on embedded systems. (micro controller have limitation on the dynamic memory allocation)
rclc_executor_t executor = rclc_executor_get_zero_initialized_executor(); //The executor manages scheduling and running callbacks (timers, subscriptions).
rcl_node_t node = rcl_get_zero_initialized_node();
rcl_timer_t timer = rcl_get_zero_initialized_timer();
rcl_publisher_t encoder_data_publisher = rcl_get_zero_initialized_publisher();

Encoder test_station_encoder(EN_CH_A, EN_CH_B);
int32_t ticks = 0;

uint32_t loop_timestamp = 0;
uint32_t loop_timestep = 2;
int32_t raw_data = 0;

// Run the function to check the "rcl return code" from the called action ex. publish, etc.
// We are looing for "RCL_RET_OK" to ensure the success execution
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){_error_handler();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void _error_handler(){
  uint32_t _error_handler_timestamp = millis();
  uint8_t _error_handler_timestep = 200;
  while(true){
    if (millis() - _error_handler_timestamp > _error_handler_timestep){
      _error_handler_timestamp = millis();
      digitalToggle(LED_PIN);
    }
  }
}

/*
  Raw quadrature encoder ticks published by micro-ROS on Teensy 4.1.
  name: EncoderRaw
  std_msgs/Header header          # ROS 2 standard header (timestamp + frame_id)
  int32   ticks                   # cumulative tick count (signed)
  float64 raw_position            # position converted from ticks [rad or m depending on setup]
  uint32  dt_us                   # microseconds since last sample
*/

void timer_cb(rcl_timer_t *timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);

  if(timer != NULL){
    int64_t time_since_last_call;
    RCSOFTCHECK(rcl_timer_get_time_since_last_call(timer, &time_since_last_call));

    // Stamp using synced epoch time
    int64_t nanos = rmw_uros_epoch_nanos();
    encoder_raw.header.stamp.sec     = (int32_t)(nanos / 1000000000LL);
    encoder_raw.header.stamp.nanosec = (uint32_t)(nanos % 1000000000LL);

    ticks = test_station_encoder.read();
    encoder_raw.ticks        = ticks;
    encoder_raw.raw_position = ticks2rad(ticks);
    // encoder_raw.dt_us        = (time_since_last_call % 1000000000) / 1000;
    encoder_raw.dt_us        = 10000;

    RCSOFTCHECK(rcl_publish(&encoder_data_publisher, &encoder_raw, NULL));
  }
}

void setup(){
  //config serial port
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  pinMode(LED_PIN, OUTPUT);
  pinMode(EN_CH_A, INPUT);
  pinMode(EN_CH_B, INPUT);
  
  //wait the initialization for two seconds.
  delay(2000);

  //create init_options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 156)); // match ROS_DOMAIN_ID on host
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
  rmw_uros_sync_session(1000);

  //create node
  RCCHECK(rclc_node_init_default(
      &node,
      "encoder_data_publisher",
      // "",                              // was root namespace
      ROS_NAMESPACE,                      // now /G<GROUP_NUMBER>
      &support
    )
  );

  //create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  
  //create publisher
  RCCHECK(rclc_publisher_init_default(
      &encoder_data_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(claude_visualizer_interface, msg, EncoderRaw),
      // "/encoder_raw"                   // was absolute (ignored the namespace)
      "encoder_raw"                       // relative → /G<GROUP_NUMBER>/encoder_raw
    )
  );

  //create timer
  const unsigned int timestep = 10; //[ms]
  RCCHECK(rclc_timer_init_default(
      &timer, 
      &support, 
      RCL_MS_TO_NS(timestep),
      timer_cb
    )
  );
  
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop(){
  // if(SERIAL_DEBUG == true){
  //   if(millis() - loop_timestamp > loop_timestep){
  //     loop_timestamp = millis();
  //     raw_data = test_station_encoder.read();
  //     Serial.println(raw_data);
  //   }
  // }
  // else{
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(20)));
  // }
}
  


