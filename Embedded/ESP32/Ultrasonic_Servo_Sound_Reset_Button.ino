Ultrasonic_Servo_Sound_Reset_Button.ino
// ================== LIBRARIES ==================
#include <Arduino.h>
#include <ESP32Servo.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>

// ================== CONSTANTS ==================
#define SOUND_SPEED 0.034  // cm/us
#define NUM_SENSORS 4
#define MIC_PIN 35
#define SAMPLE_WINDOW 50  // ms

const int trigPins[NUM_SENSORS] = { 23, 19, 18, 5 };
const int echoPins[NUM_SENSORS] = { 15, 2, 4, 17 };
const int servoPin = 32;
const int buttonPin = 33;
const int ledPin = 25;

const int STOP_US = 1500;
const int SLOW_CCW_US = 1800;
const int SLOW_CW_US  = 1200;
const int pulseDuration = 500;
const int DEGREES_PER_PULSE = 26;

bool lastButtonState = true;
int currentAngle = 0;
int lastPublishedAngle = -1;
int pulseCountFromZero = 0;
float turnValue = 0;
float voltageLast = 0.0;
float dB_level = 0.0;

// ================== ROS VARIABLES ==================
rcl_publisher_t distance_publisher;
rcl_publisher_t index_publisher;
rcl_publisher_t angle_publisher;
rcl_publisher_t pairing_publisher;
rcl_publisher_t db_publisher;

rcl_subscription_t servo_subscriber;

std_msgs__msg__Float32 distance_msg;
std_msgs__msg__Int32 index_msg;
std_msgs__msg__String angle_msg;
std_msgs__msg__Float32 pairing_msg;
std_msgs__msg__Float32 db_msg;
geometry_msgs__msg__Twist servo_msg;

rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ================== OBJECTS ==================
Servo myservo;

// ================== UTILS ==================
float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 999.0;
  return duration * SOUND_SPEED / 2.0;
}

float getClosestDistance(int &closestSensorIndex) {
  float minDistance = 999.0;
  closestSensorIndex = -1;
  for (int i = 0; i < NUM_SENSORS; i++) {
    float dist = readDistanceCM(trigPins[i], echoPins[i]);
    if (dist < minDistance) {
      minDistance = dist;
      closestSensorIndex = i;
    }
  }
  return minDistance;
}

float read_dB() {
  unsigned long startMillis = millis();
  int signalMax = 0;
  int signalMin = 4095;

  while (millis() - startMillis < SAMPLE_WINDOW) {
    int sample = analogRead(MIC_PIN);
    if (sample < 4095) {
      if (sample > signalMax) signalMax = sample;
      if (sample < signalMin) signalMin = sample;
    }
  }

  int peakToPeak = signalMax - signalMin;
  voltageLast = (peakToPeak * 3.3) / 4095.0;
  float dB = mapf(voltageLast, 0.1, 3.0, 30.0, 110.0);
  return constrain(dB, 30.0, 110.0);
}

// ================== ROS CALLBACKS ==================
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  if (timer == NULL) return;

  // Publish ultrasonic
  int sensorIndex;
  float closestDistance = getClosestDistance(sensorIndex);
  distance_msg.data = closestDistance;
  index_msg.data = sensorIndex;
  rcl_publish(&distance_publisher, &distance_msg, NULL);
  rcl_publish(&index_publisher, &index_msg, NULL);

  // Publish microphone dB
  dB_level = read_dB();
  db_msg.data = dB_level;
  rcl_publish(&
  , &db_msg, NULL);
}

void servo_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  turnValue = msg->angular.y;
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  analogReadResolution(12);  // 12-bit ADC

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(trigPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  ESP32PWM::allocateTimer(0);
  myservo.setPeriodHertz(50);
  myservo.attach(servoPin, 500, 2400);
  myservo.writeMicroseconds(STOP_US);

  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "robot_node", "", &support);

  rclc_publisher_init_default(&distance_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/Ultrasonic_Distance");
  rclc_publisher_init_default(&index_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/Ultrasonic_Index");
  rclc_publisher_init_default(&angle_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/Servo_Angle");
  rclc_publisher_init_default(&pairing_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/Pairing_Mode");
  rclc_publisher_init_default(&db_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "/Microphone_dB");

  rclc_subscription_init_default(&servo_subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/Servo_Control");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(200), timer_callback);
  rclc_executor_init(&executor, &support.context, 4, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &servo_callback, ON_NEW_DATA);
}

// ================== LOOP ==================
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  // Button press check
  bool isButtonPressed = (digitalRead(buttonPin) == LOW);
  if (isButtonPressed != lastButtonState) {
    digitalWrite(ledPin, isButtonPressed ? HIGH : LOW);
    pairing_msg.data = isButtonPressed ? 1.0 : 0.0;
    rcl_publish(&pairing_publisher, &pairing_msg, NULL);
    lastButtonState = isButtonPressed;
  }

  // Servo movement logic
  char angleBuffer[16];
  if (turnValue == 1) {
    pulseCountFromZero--;
    currentAngle -= DEGREES_PER_PULSE;
    if (currentAngle < 0) currentAngle += 360;

    myservo.writeMicroseconds(SLOW_CW_US);
    delay(pulseDuration);
    myservo.writeMicroseconds(STOP_US);
    turnValue = 0;
  }
  else if (turnValue == -1) {
    pulseCountFromZero++;
    currentAngle += DEGREES_PER_PULSE;
    if (currentAngle >= 360) currentAngle -= 360;

    myservo.writeMicroseconds(SLOW_CCW_US);
    delay(pulseDuration);
    myservo.writeMicroseconds(STOP_US);
    turnValue = 0;
  }
  else if (turnValue == 5) {
    int pulsesToUndo = abs(pulseCountFromZero);
    int direction = (pulseCountFromZero > 0) ? 1 : -1;

    for (int i = 0; i < pulsesToUndo; i++) {
      if (direction == 1) {
        myservo.writeMicroseconds(SLOW_CW_US);
        delay(pulseDuration);
        myservo.writeMicroseconds(STOP_US);
        currentAngle -= DEGREES_PER_PULSE;
        if (currentAngle < 0) currentAngle += 360;
      } else {
        myservo.writeMicroseconds(SLOW_CCW_US);
        delay(pulseDuration);
        myservo.writeMicroseconds(STOP_US);
        currentAngle += DEGREES_PER_PULSE;
        if (currentAngle >= 360) currentAngle -= 360;
      }
      delay(50);
    }

    pulseCountFromZero = 0;
    lastPublishedAngle = -1;
    turnValue = 0;
  }

  if (currentAngle != lastPublishedAngle) {
    sprintf(angleBuffer, "Angle: %d", currentAngle);
    angle_msg.data.data = angleBuffer;
    angle_msg.data.size = strlen(angleBuffer);
    angle_msg.data.capacity = angle_msg.data.size + 1;
    rcl_publish(&angle_publisher, &angle_msg, NULL);
    lastPublishedAngle = currentAngle;
  }
}