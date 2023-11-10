
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>

// Motor pins
const int leftFront = 19;
const int leftBack = 18;
const int rightFront = 5;
const int rightBack = 17;

String str;

const int enableLeft = 23;
int channel1 = 0;

const int enableRight = 16;
int channel2 = 1;

int freq = 1000;
int Res = 8;

Servo gripper;
Servo lift;
int gripperPin = 32;
int liftPin = 33;
int pos0 = 10;
int pos1 = 10;
bool gripperOpen = false;
bool liftOpen = false;

// Encoder pins
const int encoderPinA = 34;
const int encoderPinB = 35;
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;

long previousTime = 0;
float ePrevious = 0;
float eIntegral = 0;

volatile unsigned long prev_time = 0;
double rotationsA = 0;
double rotationsB = 0;
double distanceA = 0;
double distanceB = 0;
double distance = 0, prev_distance = 0;
volatile bool en1Flag;
volatile bool en2Flag;

volatile double x = 0, y = 0, angle = 0;

float kp = 2.0;
float kd = 2.0;
float ki = 2.0;

//network credentials
const char* ssid = "gado";
const char* password = "21010716*22";
String text;
//Global varialbe defined with port 80
WebSocketsServer webSocket = WebSocketsServer(80);
// Called when websocket server receives any messages
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  //Figure out the type of Websocket Event
  switch (type) {
    // Client has disconnected
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected\n", num);
      break;

    //New client has connected to the server
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from: \n", num);
        Serial.println(ip.toString());
      }
      break;
      //Echo the text messages


    case WStype_TEXT:
      //Serial.printf("[%u] Text %s\n", num, payload);
      text = String((char*)payload);
      Serial.print("this is the recieved text: ");
      Serial.println(text);
      // webSocket.broadcastTXT(payload);

      if (text == "W") {
        movingForward();
      } else if (text == "S") {
        movingBackward();
      } else if (text == "D") {
        movingRight();
      } else if (text == "A") {
        movingLeft();
      } else if (text == "G") {
        toggleGripper();
      } else if (text == "R") {
        toggleLift();
      } else if (text == "F") {
        noMovement();
      }
      break;
    // For anything else: do nothing
    case WStype_BIN:
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Connecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  //print IP Address
  Serial.println("Connected");
  Serial.print("My IP Address: ");
  Serial.println(WiFi.localIP());

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

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  gripper.setPeriodHertz(50);
  gripper.attach(gripperPin, 500, 2400);
  lift.setPeriodHertz(50);
  lift.attach(liftPin, 500, 2400);

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), handleEncoder2, RISING);

  //start Websocket Server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

long last = 0;

void loop() {
  int target = encoder1Count;
  float u = pidController(target, kp, kd, ki);

  // put your main code here, to run repeatedly:
  // Calculate speed
  unsigned long currentT = millis();
  unsigned long elapsedTime = currentT - prev_time;

  if (elapsedTime >= 1000) {
    // Calculate speed in revolutions per second
    rotationsA += (double)encoder1Count / 18.0;
    rotationsB += (double)encoder2Count / 18.0;

    //speed = (float)(distance - prev_distance) / (float)elapsedTime * 1000.0;

    // Reset pulse count and update last time
    encoder1Count = 0;
    encoder2Count = 0;
    prev_time = currentT;
  }

  // Calculate distance
  //18 slits
  distanceA = (rotationsA * M_PI * 0.65);  // Assuming a wheel diameter of 65 mm
  distanceB = (rotationsB * M_PI * 0.65);
  distance = (distanceA + distanceB) / 2.00;

  angle = angle + (distanceA - distanceB) / 19.4;
  x = x + (distance - prev_distance) * cos(angle * M_PI / 180);
  y = y + (distance - prev_distance) * sin(angle * M_PI / 180);

  prev_distance = distance;
/*
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" centimeters");
  Serial.print("Cordinates,x= ");
  Serial.println(x);
  Serial.print("Cordinates,y= ");
  Serial.println(y);
*/
  webSocket.loop();
  if (millis() > last + 50) {
    str = String   (x) + "," + String(y);
    webSocket.broadcastTXT(text);
    //Serial.println("gadoooooo");
    last = millis();
  }
}


void handleEncoder1() {
  if (en1Flag == 1) {
    encoder1Count++;
  } else {
    encoder1Count--;
  }
  //Serial.println(encoder1Count);
}

void handleEncoder2() {
  if (en2Flag == 1) {
    encoder2Count++;
  } else {
    encoder2Count--;
  }
}

// PID controller
float pidController(int target, float kp, float kd, float ki) {
  long currentTime = micros();
  float deltaT = ((float)(currentTime - previousTime)) / 1.0e6;

  int e = encoder2Count - target;  //
  float eDerivative = (e - ePrevious) / deltaT;
  eIntegral = eIntegral + e * deltaT;

  float u = (kp * e) + (ki * eIntegral) + (kd * eDerivative);

  previousTime = currentTime;
  ePrevious = e;

  return u;
}

// Motor control functions
void moveLeftMotor(int frontPin, int backPin, float u) {
  int speed = fabs(u);
  if (speed > 255) {
    speed = 255;
  }

  int direction = (u > 0) ? HIGH : LOW;

  digitalWrite(frontPin, direction);
  digitalWrite(backPin, !direction);
  ledcWrite(channel1, speed);
  Serial.println(u);
}

void moveRightMotor(int frontPin, int backPin, float u) {
  float speed = fabs(u);
  if (speed > 255) {
    speed = 255;
  }

  int direction = (u > 0) ? HIGH : LOW;

  digitalWrite(frontPin, direction);
  digitalWrite(backPin, !direction);
  ledcWrite(channel2, speed);
  Serial.println(u);
}


// Robot motion control
void movingForward() {
  int target = encoder1Count;
  float u = pidController(target, kp, kd, ki);
  
  en1Flag = 1;
  en2Flag = 1;

  moveLeftMotor(leftFront, leftBack, 255);
  moveRightMotor(rightFront, rightBack, 255);
}

void movingBackward() {
  int target = encoder1Count;
  float u = pidController(target, kp, kd, ki);
  
  en1Flag = 0;
  en2Flag = 0;

  moveLeftMotor(leftFront, leftBack, -255);
  moveRightMotor(rightFront, rightBack, -255);
}

void movingLeft() {
  int target = encoder1Count;
  float u = pidController(target, kp, kd, ki);
 
  en1Flag = 1;
  en2Flag = 0;

  moveLeftMotor(leftFront, leftBack, -255);
  moveRightMotor(rightFront, rightBack, 255);
}

void movingRight() {
  int target = encoder1Count;
  float u = pidController(target, kp, kd, ki);
  
  en1Flag = 0;
  en2Flag = 1;

  moveLeftMotor(leftFront, leftBack, 255);
  moveRightMotor(rightFront, rightBack, -255);
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
    for (int angle=10;angle<=50;angle++){
    gripper.write(angle);
    delay(10);
    }
    
    pos0 = 50;
  } else {
    for (int angle=50;angle>=10;angle--){
    gripper.write(angle);
    delay(10);
    }
    pos0 = 10;
  }
}

// Function to toggle the lift's open and closed state
void toggleLift() {
  liftOpen = !liftOpen;
  if (liftOpen) {
    for (int angle=10;angle<=50;angle++){
    lift.write(angle);
    delay(10);
    }
    pos1 = 50;
  } else {
    for (int angle=50;angle>=10;angle--){
    lift.write(angle);
    delay(10);
    }
    pos1 = 10;
  }
}
