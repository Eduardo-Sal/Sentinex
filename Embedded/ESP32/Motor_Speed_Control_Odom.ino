Motor_Speed_Control_Odom.ino
/* Motor controller using micro_ros serial set_microros_transports */
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>
#include "odometry.h"
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/float32.h>
#include <tf2_msgs/msg/tf_message.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <sensor_msgs/msg/joint_state.h>

// Pin declarations (keep these as-is for your hardware)
int8_t L_FORW = 33;
int8_t L_BACK = 25;
int8_t L_enablePin = 32;
int8_t L_encoderPin1 = 15;  // Encoder output for left wheel
int8_t L_encoderPin2 = 2;


int8_t R_FORW = 14;
int8_t R_BACK = 27;
int8_t R_enablePin = 26;
int8_t R_encoderPin1 = 0;  // Encoder output for right wheel
int8_t R_encoderPin2 = 4;


// Robot parameters (meters)
float wheels_y_distance_ = 0.22;
float wheel_radius = 0.07;
float wheel_circumference_ = 2 * 3.14 * wheel_radius;


// Encoder ticks per revolution
int tickPerRevolution_LW = 133;
int tickPerRevolution_RW = 133;
int threshold = 150;


// PID constants (tuned values)
float kp_l = 1.0; // Makes the motor respond more strongly to errors
float ki_l = 0.7; // Accumulates past error to eliminate steady-state drift
float kd_l = 4.0; // Adds braking by reacting to rate of change of error

float kp_r = 2.0;
float ki_r = 0.0;
float kd_r = 3.0;


// PWM parameters
const int freq = 30000;
const int pwmChannelL = 0;
const int pwmChannelR = 1;
const int resolution = 8;


// Global maximum speed setting (as a percentage of full PWM)
const int maxSpeedPercent = 80;  // For example, 80% of full speed


// ROS2 related variables
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t odom_publisher;
std_msgs__msg__Int32 encodervalue_l;
std_msgs__msg__Int32 encodervalue_r;
nav_msgs__msg__Odometry odom_msg;
// sensor_msgs__msg__JointState joint_state_msg;
rcl_timer_t ControlTimer;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
Odometry odometry;

rcl_subscription_t speed_subscriber;
std_msgs__msg__Float32 speed_msg;
float speedMultiplier = 1.0;  // Default to 100% speed

rcl_publisher_t tf_publisher;
rcl_publisher_t joint_state_publisher;
sensor_msgs__msg__JointState joint_state_msg;

// === Publish odom -> base_link TF ===
geometry_msgs__msg__TransformStamped odom_tf;
tf2_msgs__msg__TFMessage tf_msg;

// MotorController class definition
class MotorController {
public:
  int8_t Forward;
  int8_t Backward;
  int8_t Enable;
  int8_t EncoderPinA;
  int8_t EncoderPinB;
  std_msgs__msg__Int32 EncoderCount;
  volatile long CurrentPosition;
  volatile long PreviousPosition;
  volatile long CurrentTime;
  volatile long PreviousTime;
  volatile long CurrentTimeforError;
  volatile long PreviousTimeForError;
  float eintegral;
  float ederivative;
  float rpmPrev;
  float kp;
  float ki;
  float kd;
  float error;
  float previousError = 0;
  int tick;


  MotorController(int8_t ForwardPin, int8_t BackwardPin, int8_t EnablePin,
                  int8_t EncoderA, int8_t EncoderB, int tickPerRevolution) {
    this->Forward = ForwardPin;
    this->Backward = BackwardPin;
    this->Enable = EnablePin;
    this->EncoderPinA = EncoderA;
    this->EncoderPinB = EncoderB;
    this->tick = tickPerRevolution;
    pinMode(Forward, OUTPUT);
    pinMode(Backward, OUTPUT);
    pinMode(EnablePin, OUTPUT);
    pinMode(EncoderPinA, INPUT);
    pinMode(EncoderPinB, INPUT);
  }


  // Initialize PID parameters
  void initPID(float proportionalGain, float integralGain, float derivativeGain) {
    kp = proportionalGain;
    ki = integralGain;
    kd = derivativeGain;
  }


  // Calculate RPM using encoder ticks (no heavy filtering for faster response)
  float getRpm() {
    CurrentPosition = EncoderCount.data;
    CurrentTime = millis();
    float delta = ((float)CurrentTime - PreviousTime) / 1000.0; // seconds
    float velocity = ((float)CurrentPosition - PreviousPosition) / delta;
    float rpm = (velocity / tick) * 60;
    PreviousPosition = CurrentPosition;
    PreviousTime = CurrentTime;
    rpmPrev = rpm;
    return rpm;
  }


  // PID controller function
  float pid(float setpoint, float feedback) {
    CurrentTimeforError = millis();
    float delta = ((float)CurrentTimeforError - PreviousTimeForError) / 1000.0;
    error = setpoint - feedback;
    eintegral += error * delta;
    // Anti-windup: Clamp the integrator term
    const float max_integral = 100.0;
    if (eintegral > max_integral)
      eintegral = max_integral;
    else if (eintegral < -max_integral)
      eintegral = -max_integral;
    ederivative = (error - previousError) / delta;
    float control_signal = (kp * error) + (ki * eintegral) + (kd * ederivative);
    previousError = error;
    PreviousTimeForError = CurrentTimeforError;
    return control_signal;
  }


  // Drive the motor based on the control signal.
  // This function now limits the PWM output based on maxSpeedPercent.
  void moveBase(float ActuatingSignal, int threshold, int pwmChannel) {
    if (ActuatingSignal > 0) {
      digitalWrite(Forward, HIGH);
      digitalWrite(Backward, LOW);
    } else {
      digitalWrite(Forward, LOW);
      digitalWrite(Backward, HIGH);
    }
    int pwm = threshold + (int)fabs(ActuatingSignal);
    int max_pwm = (255 * maxSpeedPercent) / 100;  // Calculate max allowed PWM
    if (pwm > max_pwm)
      pwm = max_pwm;
    ledcWrite(pwmChannel, pwm);
  }


  // Stop the motor by clearing the PWM output and disabling the driver
  void stop(int pwmChannel) {
    digitalWrite(Forward, LOW);
    digitalWrite(Backward, LOW);
    ledcWrite(pwmChannel, 0);
  }
};


// Create objects for left and right wheels
MotorController leftWheel(L_FORW, L_BACK, L_enablePin, L_encoderPin1, L_encoderPin2, tickPerRevolution_LW);
MotorController rightWheel(R_FORW, R_BACK, R_enablePin, R_encoderPin1, R_encoderPin2, tickPerRevolution_RW);


#define LED_PIN 2
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if (temp_rc != RCL_RET_OK) { error_loop(); } }


// Error loop: Blink LED to signal errors (useful for headless operation)
void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


// Subscription callback for incoming cmd_vel messages
void subscription_callback(const void* msgin) {
  const geometry_msgs__msg__Twist* twist_msg = (const geometry_msgs__msg__Twist*)msgin;
  msg = *twist_msg;
  prev_cmd_time = millis();
  // Reset PID integrators and previous errors on new command
  leftWheel.eintegral = 0;
  rightWheel.eintegral = 0;
  leftWheel.previousError = 0;
  rightWheel.previousError = 0;
}

void speed_callback(const void *msgin) {
  const std_msgs__msg__Float32 *msg_in = (const std_msgs__msg__Float32 *)msgin;
  speedMultiplier = msg_in->data;
}


// Timer callback that controls the motors and computes odometry
void MotorControll_callback(rcl_timer_t* timer, int64_t last_call_time) {
  geometry_msgs__msg__Twist current_cmd = msg;
  float linearVelocity = current_cmd.linear.x;
  float angularVelocity = current_cmd.angular.z;
  
  // Get current wheel RPMs
  float currentRpmL = leftWheel.getRpm();
  float currentRpmR = rightWheel.getRpm();

  bool timeout = (millis() - prev_cmd_time > 500);

  // Use half the wheel track for the differential drive calculations
  float halfBase = wheels_y_distance_ / 2.0;
  float vL = (linearVelocity - (angularVelocity * halfBase)) * 20 * speedMultiplier;
  float vR = (linearVelocity + (angularVelocity * halfBase)) * 20 * speedMultiplier;


  // Reset PID integrators if a significant change is detected
  static float prev_vL = 0.0, prev_vR = 0.0;
  const float threshold_change = 0.5;
  if (fabs(vL - prev_vL) > threshold_change) {
    leftWheel.eintegral = 0.0;
    leftWheel.previousError = 0.0;
  }
  if (fabs(vR - prev_vR) > threshold_change) {
    rightWheel.eintegral = 0.0;
    rightWheel.previousError = 0.0;
  }
  prev_vL = vL;
  prev_vR = vR;

  if (!timeout)
  {

    // Compute PID control signals
    float actuating_signal_LW = leftWheel.pid(vL, currentRpmL);
    float actuating_signal_RW = rightWheel.pid(vR, currentRpmR);


    // Constrain control signals to safe limits
    actuating_signal_LW = constrain(actuating_signal_LW, -100, 100);
    actuating_signal_RW = constrain(actuating_signal_RW, -100, 100);


    // If command is zero, stop the motors
    if (vL == 0 && vR == 0) {
      leftWheel.stop(pwmChannelL);
      rightWheel.stop(pwmChannelR);
      actuating_signal_LW = 0;
      actuating_signal_RW = 0;
    } else {
      rightWheel.moveBase(actuating_signal_RW, threshold, pwmChannelR);
      leftWheel.moveBase(actuating_signal_LW, threshold, pwmChannelL);
    }
  }


  // Still compute and publish odometry even if stopped
  if (timeout) {
    leftWheel.stop(pwmChannelL);
    rightWheel.stop(pwmChannelR);
    msg.linear.x = 0.0;
    msg.angular.z = 0.0;
    linearVelocity = 0.0;
    angularVelocity = 0.0;
  }

  // Odometry calculation
  float average_rps_x = ((float)(currentRpmL + currentRpmR) / 2) / 60.0;
  float linear_x = average_rps_x * wheel_circumference_;
  float average_rps_a = ((float)(-currentRpmL + currentRpmR) / 2) / 60.0;
  float angular_z = (average_rps_a * wheel_circumference_) / (wheels_y_distance_ / 2.0);
  float linear_y = 0;
  unsigned long now = millis();
  float vel_dt = (now - prev_odom_update) / 1000.0;
  prev_odom_update = now;
  odometry.update(vel_dt, linear_x, linear_y, angular_z);
 
  // Publish odometry data
  odom_msg = odometry.getData();
  struct timespec time_stamp = {0};
  unsigned long t = millis();
  time_stamp.tv_sec = t / 1000;
  time_stamp.tv_nsec = (t % 1000) * 1000000;
  odom_msg.header.stamp.sec = time_stamp.tv_sec;
  odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));

  // static bool tf_id_initialized = false;
  // if (!tf_id_initialized) {
  //     rosidl_runtime_c__String__init(&odom_tf.header.frame_id);
  //     rosidl_runtime_c__String__init(&odom_tf.child_frame_id);
  //     tf_id_initialized = true;
  // }

  // rosidl_runtime_c__String__assign(&odom_tf.header.frame_id, "odom");
  // rosidl_runtime_c__String__assign(&odom_tf.child_frame_id, "base_link");

  // Use same timestamp as odometry
  odom_tf.header.stamp = odom_msg.header.stamp;

  // Fill in the transform data
  odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation = odom_msg.pose.pose.orientation;

  // Pack into TFMessage and publish
  tf_msg.transforms.size = 1;
  tf_msg.transforms.data = &odom_tf;

  RCSOFTCHECK(rcl_publish(&tf_publisher, &tf_msg, NULL));

  // Set joint positions using encoder ticks
  joint_state_msg.position.data[0] = (double)leftWheel.EncoderCount.data / tickPerRevolution_LW * 2.0 * M_PI;
  joint_state_msg.position.data[1] = (double)rightWheel.EncoderCount.data / tickPerRevolution_RW * 2.0 * M_PI;

  joint_state_msg.header.stamp = odom_msg.header.stamp;

  RCSOFTCHECK(rcl_publish(&joint_state_publisher, &joint_state_msg, NULL));

}


// Encoder interrupt for left wheel
void updateEncoderL() {
  if (digitalRead(leftWheel.EncoderPinB) > digitalRead(leftWheel.EncoderPinA))
    leftWheel.EncoderCount.data++;
  else
    leftWheel.EncoderCount.data--;
  encodervalue_l = leftWheel.EncoderCount;
}


// Updated encoder interrupt for right wheel (using the same ordering as left)
void updateEncoderR() {
  if (digitalRead(rightWheel.EncoderPinB) > digitalRead(rightWheel.EncoderPinA))
    rightWheel.EncoderCount.data++;
  else
    rightWheel.EncoderCount.data--;
  encodervalue_r = rightWheel.EncoderCount;
}


void setup() {
  // Initialize PID parameters
  leftWheel.initPID(kp_l, ki_l, kd_l);
  rightWheel.initPID(kp_r, ki_r, kd_r);


  // Set up encoder interrupts (ensure these pins support interrupts on the ESP32)
  attachInterrupt(digitalPinToInterrupt(leftWheel.EncoderPinB), updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(rightWheel.EncoderPinB), updateEncoderR, RISING);


  // Set up PWM channels
  ledcSetup(pwmChannelL, freq, resolution);
  ledcAttachPin(leftWheel.Enable, pwmChannelL);
  ledcSetup(pwmChannelR, freq, resolution);
  ledcAttachPin(rightWheel.Enable, pwmChannelR);


  // Initialize micro-ROS transports
  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(2000);  // Allow time for the micro-ROS agent to be ready


  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_esp32_node", "", &support));


  // Create subscriber for /Motor_Control topic
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/Motor_Control"));

  // Create subscriber for /Speed_Control topic
  RCCHECK(rclc_subscription_init_default(
    &speed_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/Speed_Control"));

  // Create odometry publisher
  RCCHECK(rclc_publisher_init_default(
    &odom_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "/odom/unfiltered"));

  RCCHECK(rclc_publisher_init_default(
    &tf_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
    "/tf"));

  RCCHECK(rclc_publisher_init_default(
    &joint_state_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "/joint_states"));

  rosidl_runtime_c__String__init(&joint_state_msg.header.frame_id);
  rosidl_runtime_c__String__assign(&joint_state_msg.header.frame_id, "base_link");

  joint_state_msg.name.capacity = 2;
  joint_state_msg.name.size = 2;  
  joint_state_msg.name.data = (rosidl_runtime_c__String*)malloc(sizeof(rosidl_runtime_c__String) * 2);
  rosidl_runtime_c__String__init(&joint_state_msg.name.data[0]);
  rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], "left_wheel_joint");
  rosidl_runtime_c__String__init(&joint_state_msg.name.data[1]);
  rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1], "right_wheel_joint");

  joint_state_msg.position.capacity = 2;
  joint_state_msg.position.size = 2;
  joint_state_msg.position.data = (double*)malloc(sizeof(double) * 2);

  // Initialize frame_id and child_frame_id for odometry
  rosidl_runtime_c__String__init(&odom_msg.header.frame_id);
  rosidl_runtime_c__String__init(&odom_msg.child_frame_id);
  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");

  rosidl_runtime_c__String__init(&odom_tf.header.frame_id);
  rosidl_runtime_c__String__init(&odom_tf.child_frame_id);
  rosidl_runtime_c__String__assign(&odom_tf.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&odom_tf.child_frame_id, "base_link");

  // Initialize joint_state_msg.header.frame_id
  // rosidl_runtime_c__String__init(&joint_state_msg.header.frame_id);
  // rosidl_runtime_c__String__assign(&joint_state_msg.header.frame_id, "base_link");

  // Initialize motor control timer (10ms period)
  const unsigned int samplingT = 10;
  RCCHECK(rclc_timer_init_default(
    &ControlTimer,
    &support,
    RCL_MS_TO_NS(samplingT),
    MotorControll_callback));


  // Create executor and add subscription and timer
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &speed_subscriber, &speed_msg, &speed_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &ControlTimer));
}


void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(100);
}