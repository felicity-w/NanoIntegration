/*
  Arduino Starter Kit example
  Project 10 - Zoetrope

  This sketch is written to accompany Project 10 in the Arduino Starter Kit

  Parts required:
  - two 10 kilohm resistors
  - two momentary pushbuttons
  - one 10 kilohm potentiometer
  - motor
  - 9V battery
  - H-Bridge

  created 13 Sep 2012
  by Scott Fitzgerald
  Thanks to Federico Vanzati for improvements

  https://store.arduino.cc/genuino-starter-kit

  This example code is part of the public domain.
*/



int LED = 4;
const int mosfetgate = 13;               // connected to pin 13 for the gate of the N-type mosfet
int volt_1000 = A1;
// int volt_100 = A2;;
// int volt_11 = A1;
int volt_circuit = A0;

float raw_volt_1000;
float raw_volt_100;
float raw_volt_11;
float raw_volt_circuit;
float act_volt_1000;
float act_volt_100;
float act_volt_11;
float act_volt_circuit;
float current_1000;
float current_100;
float current_11;
float rail_voltage = 18;
float point = 1024.0;
float current_setpoint = 300; // In microamps

float previousReadTime = 0;
float previousWriteTime = 0;
float mosfetOnTime = 100;   // LED on time in microseconds
float mosfetOffTime = 10000;  // LED off time in microseconds
bool mosfetState = LOW;
bool LEDState = LOW;
bool mosfet1000State = HIGH;

// Constants
const float Kp = 1.0;  // Proportional gain
const float Ki = 0.5;  // Integral gain
const float Kd = 0.2;  // Derivative gain

// Variables
float setpoint = 50.0;  // Desired setpoint must be between 150 and 800
float time_setpoint = 0;
float time_input = 0;
float input = 0.0;      // Actual system measurement
float output = 0.0;     // Control output

float error = 0.0;      // Error (difference between setpoint and input)
float lastError = 0.0;  // Previous error
float proportional = 0.0;
float integral = 0.0;   // Integral term (accumulated error)
float derivative = 0.0; // Derivative term (rate of change of error)

void setup() {
  // initialize the inputs and outputs
  pinMode(LED, OUTPUT);
  pinMode(mosfetgate, OUTPUT);
  pinMode(volt_1000, INPUT);
  // pinMode(volt_100, INPUT);
  // pinMode(volt_11, INPUT);
  pinMode(volt_circuit, INPUT);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(2000000);
}

void loop() {
  // Analog read at 1ms rate
  float currentReadTime = micros();
  if (currentReadTime - previousReadTime >= 1000) { //setting the timing of how frequent you read the value
    previousReadTime = currentReadTime;

    // read values from the sensor:
    raw_volt_1000 = (float)analogRead(volt_1000);
    act_volt_1000 = (raw_volt_1000 * rail_voltage) / point;
    // raw_volt_100 = (float)analogRead(volt_1000);
    // act_volt_100 = (raw_volt_100 * rail_voltage) / point;
    // raw_volt_11 = (float)analogRead(volt_1000);
    // act_volt_11 = (raw_volt_11 * rail_voltage) / point;
    raw_volt_circuit = (float)analogRead(volt_circuit);
    act_volt_circuit = (raw_volt_circuit * rail_voltage) / point;
    current_1000 = act_volt_1000 / 1000;
    // current_100 = act_volt_100 / 100;
    // current_11 = act_volt_11 / 11;
    Serial.print(act_volt_circuit * 100); // Print the voltage before the 1k resistor
    Serial.print(",");
    Serial.println(current_1000 * 1000000, 3); // Print the voltage of the circuit - 1K resistance? in uA
    // Serial.println(current_100); // Print the voltage of the circuit - 1K resistance?
    // Serial.print(",");
    // Serial.println(current_11); // Print the voltage of the circuit - 1K resistance?
    // Serial.print(",");
    time_setpoint = 54664 * pow(2.71828, -0.005 * current_setpoint) + 0;
    time_input = 54664 * pow(2.71828, -0.005 * current_1000 * 1000000) + 0;
    error = time_setpoint - time_input;
    proportional = Kp * error;
    integral += Ki * error;
    derivative = Kd * (error - lastError);
    output = proportional + integral + derivative;
    mosfetOffTime = output;
    lastError = error;
    // Serial.println(',');
    // Serial.println(error, 6); // Print the voltage of the circuit - 1K resistance? in uA
    // Serial.println(',');
  }

  // LED control
  float currentWriteTime = micros();
  if (mosfetState == LOW) {
    // LED is currently off
    if (currentWriteTime - previousWriteTime >= mosfetOffTime) {
      previousWriteTime = currentWriteTime;

      // Turn on the MOSFET
      digitalWrite(mosfetgate, HIGH);
      mosfetState = HIGH;

      // Turn the LED ON
      digitalWrite(LED, HIGH);
      LEDState = HIGH;

      // Update the LED on time based on the dynamic value
      // mosfetOnTime = dynamicValue;
    }
  } else {
    // LED is currently on
    if (currentWriteTime - previousWriteTime >= mosfetOnTime) {
      previousWriteTime = currentWriteTime;

      // Turn off the MOSFET
      digitalWrite(mosfetgate, LOW);
      mosfetState = LOW;

      // Turn the LED ON
      digitalWrite(LED, LOW);
      LEDState = LOW;

      // Update the LED off time based on the dynamic value
      // ledOffTime = dynamicValue;
    }
  }
}
