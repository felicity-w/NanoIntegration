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



int LED = 8;
float switchingfrequency = 0.1;             // In Hertz
const int mosfetgate = 13;               // connected to pin 13 for the gate of the N-type mosfet
// const int mosfetgate10 = 12;             // connected to pin 12 for the gate of the N-type mosfet
// const int mosfetgate100 = 11;            // connected to pin 11 for the gate of the N-type mosfet
// const int mosfetgate1000 = 10;           // connected to pin 10 for the gate of the N-type mosfet  
int check_volt = A1;
int potential_divider = A0;


float period = 1000/switchingfrequency;          // Getting the time for each cycle
float dutyCycle = 0.5;                        // Duty Cycle
float on_time = period * dutyCycle; 
float off_time = period * ( 1 - dutyCycle);
float checkValue;
float checkVoltage;
float potentialValue;
float potentialVoltage;
float rail_voltage = 5;
float point = 1024.0;

float previousReadTime = 0;
float previousWriteTime = 0;
float mosfetOnTime = 10;   // LED on time in milliseconds
float mosfetOffTime = 5000;  // LED off time in milliseconds
bool mosfetState = LOW;
bool LEDState = LOW;


void setup() {
  // initialize the inputs and outputs
  pinMode(LED, OUTPUT);
  pinMode(mosfetgate, OUTPUT);
  pinMode(check_volt, INPUT);
  pinMode(potential_divider, INPUT);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(2000000);
}

void loop() {
  // Analog read at 1ms rate
  float currentReadTime = micros();
  if (currentReadTime - previousReadTime >= 1000) { //setting the timing of how frequent you read the value
    previousReadTime = currentReadTime;

    // read values from the sensor:
    checkValue = (float)analogRead(check_volt);
    checkVoltage = (checkValue * rail_voltage) / point ;
    potentialValue = (float)analogRead(potential_divider);
    potentialVoltage = (potentialValue * rail_voltage) / point ;
    Serial.println(checkVoltage); // Print the value to the serial monitor
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