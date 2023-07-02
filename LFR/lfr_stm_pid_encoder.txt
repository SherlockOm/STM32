// Define the pin connections for the IR sensor array
#define IR1 PA0
#define IR2 PA1
#define IR3 PA2
#define IR4 PA3
#define IR5 PA4

// Define the pin connections for the motor driver
#define motor1A PB0
#define motor1B PB1
#define motor2A PB6
#define motor2B PB7

// Define the pin connections for the motor encoder
#define encoder1A PC0
#define encoder1B PC1
#define encoder2A PC2
#define encoder2B PC3

// Encoder variables
volatile int count1 = 0;
volatile int count2 = 0;

// PID Constants
double Kp = 1.0;  // Proportional gain
double Ki = 0.0;  // Integral gain
double Kd = 0.0;  // Derivative gain

// PID Variables
double error = 0.0;
double integral = 0.0;
double derivative = 0.0;
double previous_error = 0.0;
double output = 0.0;

// Setpoint
double setpoint = 2.0;  // Middle sensor (IR3) should be on the line

void setup() {
  // Set up the GPIO pins for the IR sensor array as input pins with pull-up resistors
  pinMode(IR1, INPUT_PULLUP);
  pinMode(IR2, INPUT_PULLUP);
  pinMode(IR3, INPUT_PULLUP);
  pinMode(IR4, INPUT_PULLUP);
  pinMode(IR5, INPUT_PULLUP);

  // Set up the GPIO pins for the motor driver as output pins
  pinMode(motor1A, OUTPUT);
  pinMode(motor1B, OUTPUT);
  pinMode(motor2A, OUTPUT);
  pinMode(motor2B, OUTPUT);

  // Set up the GPIO pins for the motor encoder as input pins with interrupts
  pinMode(encoder1A, INPUT);
  pinMode(encoder1B, INPUT);
  pinMode(encoder2A, INPUT);
  pinMode(encoder2B, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoder1A), handleEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2A), handleEncoder2, CHANGE);
}

void loop() {
  // Read the digital values from each IR sensor
  int ir1 = digitalRead(IR1);
  int ir2 = digitalRead(IR2);
  int ir3 = digitalRead(IR3);
  int ir4 = digitalRead(IR4);
  int ir5 = digitalRead(IR5);

  // Calculate the error based on the sensor readings
  error = ir3 - setpoint;

  // Calculate the integral term
  integral += error;

  // Calculate the derivative term
  derivative = error - previous_error;
  previous_error = error;

  // Calculate the PID output
  output = Kp * error + Ki * integral + Kd * derivative;

  // Adjust the motor speeds based on the PID output
  double leftSpeed = 0.5 + output;  // Base speed + PID output
  double rightSpeed = 0.5 - output; // Base speed - PID output

  // Apply the motor speeds to the motor driver
  if (leftSpeed > 0.0) {
    analogWrite(motor1A, leftSpeed * 255);  // Scale speed to 0-255
    analogWrite(motor1B, 0);
  } else {
    analogWrite(motor1A, 0);
    analogWrite(motor1B, -leftSpeed * 
