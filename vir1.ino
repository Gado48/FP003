
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>

// Motor pins
const int leftFront = 19;   // Front left motor control pin
const int leftBack = 18;    // Back left motor control pin
const int rightFront = 5;   // Front right motor control pin
const int rightBack = 17;   // Back right motor control pin

const int enableLeft = 23;  // PWM control pin for the left motor
const int enableRight = 16; // PWM control pin for the right motor
int channel1 = 0;            // PWM channel for the left motor
int channel2 = 1;            // PWM channel for the right motor

const int freq = 1000;       // PWM frequency
const int Res = 8;           // PWM resolution

Servo gripper;                // Gripper servo motor control
Servo lift;                   // Lift servo motor control
int gripperPin = 32;          // Gripper servo control pin
int liftPin = 33;             // Lift servo control pin
int pos0 = 0;                 // Initial position of gripper servo
int pos1 = 0;                 // Initial position of lift servo
bool gripperOpen = false;     // Flag to track gripper state (open or closed)
bool liftOpen = false;        // Flag to track lift state (open or closed)

// Encoder pins
const int encoderPinA = 34;   // Encoder A signal pin
const int encoderPinB = 35;   // Encoder B signal pin
volatile long encoder1Count = 0; // Counter for the left wheel encoder
volatile long encoder2Count = 0; // Counter for the right wheel encoder

// PID controller constants
float kp = 2.0;              // Proportional gain
float kd = 2.0;              // Derivative gain
float ki = 2.0;              // Integral gain
float ePrevious = 0;         // Previous error for derivative control
float eIntegral = 0;         // Integral of the error
long previousTime = 0;       // Time for calculating the time difference in PID loop

// Network credentials
const char* ssid = "gado";      // Wi-Fi network SSID
const char* password = "21010716*22"; // Wi-Fi password
String text;                          // Variable to store received WebSocket text

// Global WebSocket object
WebSocketsServer webSocket = WebSocketsServer(80);

// WebSocket event handling function
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from: %s\n", num, ip.toString().c_str());
      }
      break;
    case WStype_TEXT:
      text = String((char*)payload);
      Serial.print("Received text: ");
      Serial.println(text);
      webSocket.broadcastTXT(payload);
      handleWebSocketCommands(text);
      break;
    default:
      break;
  }
}

// Setup function
void setup() {
  Serial.begin(115200);
  Serial.println("Connecting to Wi-Fi");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");
  Serial.print("My IP Address: ");
  Serial.println(WiFi.localIP());

  // Motor pin configuration
  pinMode(leftFront, OUTPUT);
  pinMode(leftBack, OUTPUT);
  pinMode(rightFront, OUTPUT);
  pinMode(rightBack, OUTPUT);
  pinMode(enableRight, OUTPUT);
  pinMode(enableLeft, OUTPUT);

  ledcSetup(channel1, freq, Res);
  ledcSetup(channel2, freq, Res);
  ledcAttachPin(enableRight, channel2);
  ledcAttachPin(enableLeft, channel1);

  // Initialize servo motors
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  gripper.setPeriodHertz(50);
  gripper.attach(gripperPin, 500, 2400);
  lift.setPeriodHertz(50);
  lift.attach(liftPin, 500, 2400);

  // Encoder pin setup
  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), handleEncoder2, RISING);

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

// Main loop function
void loop() {
  int target = encoder1Count;
  float u = pidController(target);

  webSocket.loop();

  // Broadcast a message periodically
  if (millis() > previousTime + 50) {
    webSocket.broadcastTXT("Broadcasting message");
    previousTime = millis();
  }
}

// Interrupt service routine for encoder A signal
void handleEncoder1() {
  encoder1Count++;
}

// Interrupt service routine for encoder B signal
void handleEncoder2() {
  encoder2Count++;
}

// PID controller function
float pidController(int target) {
  long currentTime = micros();
  float deltaT = (currentTime - previousTime) / 1.0e6;

  int e = encoder2Count - target;
  float eDerivative = (e - ePrevious) / deltaT;
  eIntegral += e * deltaT;

  float u = (kp * e) + (ki * eIntegral) + (kd * eDerivative);

  previousTime = currentTime;
  ePrevious = e;

  return u;
}

// Function to control motor speed and direction
void moveMotor(int frontPin, int backPin, float u, int channel) {
  int speed = abs(u);
  if (speed > 255) {
    speed = 255;
  }
  int direction = (u > 0) ? HIGH : LOW;

  digitalWrite(frontPin, direction);
  digitalWrite(backPin, !direction);
  ledcWrite(channel, speed);
}

// Function to handle WebSocket commands
void handleWebSocketCommands(String command) {
  if (command == "W") {
    movingForward();
  } else if (command == "S") {
    movingBackward();
  } else if (command == "D") {
    movingRight();
  } else if (command == "A") {
    movingLeft();
  } else if (command == "G") {
    toggleGripper();
  } else if (command == "R") {
    toggleLift();
  } else if (command == "F") {
    noMovement();
  }
}

// Functions for robot movement
void movingForward() {
  int target = encoder1Count;
  float u = pidController(target);
  moveMotor(leftFront, leftBack, 255 + u, channel1);
  moveMotor(rightFront, rightBack, 255 - u, channel2);
}

void movingBackward() {
  int target = encoder1Count;
  float u = pidController(target);
  moveMotor(leftFront, leftBack, 255 - u, channel1);
  moveMotor(rightFront, rightBack, 255 + u, channel2);
}

void movingLeft() {
  moveMotor(leftFront, leftBack, 255, channel1);
  moveMotor(rightFront, rightBack, 255, channel2);
}

void movingRight() {
  moveMotor(leftFront, leftBack, 255, channel1);
  moveMotor(rightFront, rightBack, 255, channel2);
}

void noMovement() {
  digitalWrite(leftFront, LOW);
  digitalWrite(leftBack, LOW);
  digitalWrite(rightFront, LOW);
  digitalWrite(rightBack, LOW);
}

// Function to toggle the gripper's open and closed state
void toggleGripper() {
  gripperOpen = !gripperOpen;
  if (gripperOpen) {
    gripper.write(50);
    pos0 = 50;
  } else {
    gripper.write(0);
    pos0 = 0;
  }
}

// Function to toggle the lift's open and closed state
void toggleLift() {
  liftOpen = !liftOpen;
  if (liftOpen) {
    lift.write(50);
    pos1 = 50;
  } else {
    lift.write(0);
    pos1 = 0;
  }
}
