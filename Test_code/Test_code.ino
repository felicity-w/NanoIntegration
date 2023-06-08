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

// const int mosfetgate = 13;                // connected to pin 13 for the gate of the N-type mosfet
// const int mosfetgate10 = 12;                // connected to pin 12 for the gate of the N-type mosfet
// const int mosfetgate100 = 11;                // connected to pin 11 for the gate of the N-type mosfet
// const int mosfetgate1000 = 10;                // connected to pin 10 for the gate of the N-type mosfet
int LED = 8;
float switchingfrequency = 0.1;        // In Hertz
int mosfetgate = 13;                // connected to pin 13 for the gate of the N-type mosfet
int mosfetgate10 = 12;
int pmos = A1;
int potential_divider = A0;


float period = 1/switchingfrequency;          // Getting the time for each cycle
float dutyCycle = 0.5;                        // Duty Cycle
float on_time = period * dutyCycle; 
float off_time = period * ( 1 - dutyCycle);
float pmosValue;
float pmosVoltage;
float potentialValue;
float potentialVoltage;
float rail_voltage = 5.04;
float point = 1024.0;



void setup() {
  // initialize the inputs and outputs
  pinMode(LED, OUTPUT);
  pinMode(pmos, INPUT);
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

void loop() {
  // read value from the sensor:
  pmosValue = (float)analogRead(pmos);
  pmosVoltage = (pmosValue * rail_voltage) / point ;
  potentialValue = (float)analogRead(pmos);
  potentialVoltage = (pmosValue * rail_voltage) / point ;
  // read the voltages   
  digitalWrite(mosfetgate, HIGH); // turn the MOSFET on
  digitalWrite(LED, HIGH); // turn the MOSFET on
  Serial.println(pmosVoltage); // Print the value to the serial monitor
  delay(1000);

  pmosValue = (float)analogRead(pmos);
  pmosVoltage = (pmosValue * rail_voltage) / point ;
  potentialValue = (float)analogRead(pmos);
  potentialVoltage = (pmosValue * rail_voltage) / point ;
  digitalWrite(mosfetgate, LOW); // turn the MOSFET on
  digitalWrite(LED, LOW); // turn the MOSFET on
  Serial.println(pmosVoltage); // Print the value to the serial monitor
  delay(1000);
  
}
