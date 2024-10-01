#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rosidl_runtime_c/string_functions.h>
#include <micro_ros_utilities/string_utilities.h>
#include <std_msgs/msg/empty.h>
#include <std_msgs/msg/string.h>

rcl_publisher_t state_publisher;
rcl_subscription_t attach_subscriber;
rcl_subscription_t detach_subscriber;

std_msgs__msg__Empty empty_msg;
std_msgs__msg__String attach_msg;
std_msgs__msg__String detach_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

#define SWITCH_PIN 2
volatile bool switchState = HIGH; // Store the current switch state
const unsigned long debounceDelay = 50; // Debounce delay in milliseconds
IntervalTimer myTimer;             // Create an interval timer
unsigned long lastDebounceTime = 0; // Timestamp of the last switch state change
volatile bool timerRunning = false;         // Flag to indicate if the timer is running
elapsedMillis sinceLastLoop;


void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void attach_subscription_callback(const void * msgin)
{ 
  RCLC_UNUSED(msgin);
  digitalWrite(LED_BUILTIN, HIGH);  
}

void detach_subscription_callback(const void * msgin)
{ 
  RCLC_UNUSED(msgin);
  digitalWrite(LED_BUILTIN, LOW);  
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Publish either attach_msg or detach_msg based on the random value
    if (switchState) {
      RCSOFTCHECK(rcl_publish(&state_publisher, &attach_msg, NULL));
    } else {
      RCSOFTCHECK(rcl_publish(&state_publisher, &detach_msg, NULL));
    }
  }
}

void handleSwitchChange() {
    unsigned long currentTime = millis(); // Get the current time

    // Check if the debounce delay has passed
    if (currentTime - lastDebounceTime > debounceDelay) {
      startTimer();
      lastDebounceTime = currentTime; // Update the last debounce time
    }
}

// Interrupt service routine for starting the timer
void startTimer() {
    // Start the timer if it's not already running
    if (!timerRunning) {
        myTimer.begin(timerCallback, debounceDelay * 1000); // Convert to microseconds
        timerRunning = true;  
    }
}

// Timer callback function
void timerCallback() {
    // Stop the timer after handling the switch state
    myTimer.end();
    timerRunning = false;  
    // Read the state of the switch
    switchState = digitalRead(SWITCH_PIN);    
}

void setup() {
  pinMode(SWITCH_PIN, INPUT_PULLUP);     // Set the switch pin as input with internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), startTimer, CHANGE);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); 

  set_microros_transports();

  // Initialize attach message
  attach_msg.data.data = (char *)malloc(10); // Allocate memory for the message
  attach_msg.data.size = strlen("attached");
  attach_msg.data.capacity = 10; // Set capacity to avoid memory issues
  strcpy(attach_msg.data.data, "attached");

  detach_msg.data.data = (char *)malloc(10); // Allocate memory for the message
  detach_msg.data.size = strlen("attached");
  detach_msg.data.capacity = 10; // Set capacity to avoid memory issues
  strcpy(detach_msg.data.data, "detached");
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "mag_dock", "", &support));

  // create attach subscriber
  RCCHECK(rclc_subscription_init_default(
    &attach_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "attach"));

  // create detach subscriber
  RCCHECK(rclc_subscription_init_default(
    &detach_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Empty),
    "detach"));

  // create state publisher
  RCCHECK(rclc_publisher_init_default(
    &state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "attached_state"));
  
  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &attach_subscriber, &empty_msg, &attach_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &detach_subscriber, &empty_msg, &detach_subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  if (sinceLastLoop > 100) {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    sinceLastLoop = 0;
  }
}
