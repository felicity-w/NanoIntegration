#include <eRCaGuy_NewAnalogRead.h>

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

//include the library
#include <eRCaGuy_NewanalogRead.h>

const int mosfetgate = 11;                // connected to pin 13 for the gate of the N-type mosfet
// const int resistorA1Pin = A0;              // connected to the 1 ohm resistor before the resistor
// const int resistorA2Pin = A1;              // connected to the 1 ohm resistor after the resistor
// const int resistorB1Pin = A2;              // connected to the lower 1ohm resistor before the resistor
// const int mosfetPin = A3; 
const float inputvoltage = 5.00;            // Input voltage of the circuit
const int switchingfrequency = 100;        // In Hertz
const float resistorA = 10;                  // Resistor value
const float resistorB = 10;
float period = 1/switchingfrequency;          // Getting the time for each cycle
float dutyCycle = 0.5;                        // Duty Cycle
int counter = 0;                            // a counter variable to count the loops
String p1 = ";";
int bits_of_precision = 16; //must be a value between 10 and 21
int num_samples = 1;

//Global variables for oversampling
byte resistorA1Pin = A0;              // connected to the 1 ohm resistor before the resistor
byte resistorA2Pin = A1;              // connected to the 1 ohm resistor after the resistor
byte resistorB1Pin = A2;              // connected to the lower 1ohm resistor before the resistor
byte mosfetPin = A3; 
byte bitsOfResolution = 14; //commanded oversampled resolution
unsigned long numSamplesToAvg = 1; //number of samples AT THE OVERSAMPLED RESOLUTION that you want to take and average
ADC_prescaler_t ADCSpeed = ADC_FAST;
/*Speed options to store into ADCSpeed are as follows:
  ADC_PRESCALER_128_CLOCK_125KHZ    
    ADC_DEFAULT (same as above)                       
    ADC_SLOW (same as above)                          
  ADC_PRESCALER_64_CLOCK_250KHZ     
  ADC_PRESCALER_32_CLOCK_500KHZ     
  ADC_PRESCALER_16_CLOCK_1MHZ       
    ADC_FAST (same as above)                          
  CAUTION_ADC_PRESCALER_8_CLOCK_2MHZ
  CAUTION_ADC_PRESCALER_4_CLOCK_4MHZ
  CAUTION_ADC_PRESCALER_2_CLOCK_8MHZ
NB: if you change the ADC clock speed, it doesn't just affect this library, it also affects the 
ADC sample rate when calling the standard core Arduino analogRead() function.
*/

// PID controller constants
float Kp = 1.0; // proportional gain
float Ki = 0.1; // integral gain
float Kd = 0.01; // derivative gain

// PID controller variables
float setpoint = 50.0; // desired setpoint
float output = 0.0; // system output
float error = 0.0; // error signal
float prevError = 0.0; // previous error signal
float integral = 0.0; // integral term
float derivative = 0.0; // derivative term
float input = 0.0; // system input

void setup() {
  // initialize the inputs and outputs
  pinMode(resistorA1Pin, INPUT);
  pinMode(resistorA2Pin, INPUT);
  pinMode(resistorB1Pin, INPUT);
  pinMode(mosfetPin, INPUT);
  pinMode(mosfetgate, OUTPUT);

  // initialize serial communication at 9600 bits per second:
  Serial.begin(1200);

  //Configure the ADC 
  adc.setADCSpeed(ADCSpeed);
  adc.setBitsOfResolution(bitsOfResolution);
  adc.setNumSamplesToAvg(numSamplesToAvg);

}

void loop() {
  // read the voltages   

  //local variables
  unsigned long analogReading;

  digitalWrite(mosfetgate, HIGH); // turn the MOSFET on
  float voltageA1 = adc.newAnalogRead(resistorA1Pin)*inputvoltage/adc.getMaxPossibleReading(); //voltage at the front of resistor A
  float voltageA2 = adc.newAnalogRead(resistorA2Pin)*inputvoltage/adc.getMaxPossibleReading(); //voltage at the back of resistor A
  // float voltageB1 = adc.newAnalogRead(resistorB1Pin)*inputvoltage/adc.getMaxPossibleReading(); //voltage at the front of resistor B
  // float voltageM = adc.newAnalogRead(mosfetPin)*inputvoltage/adc.getMaxPossibleReading(); //voltage at the front of MOSFET gate
  float currentA = (voltageA1 - voltageA2)/resistorA; // Current through resistor A
  // float currentB = voltageB1/resistorB; //Current through resistor B
  // float currentDiff = currentA - currentB; //current difference, should be 0
  // Serial.print("CurrentA: ");
  // Serial.println(currentA*1000,6); //displays current through resistor A in mA
  Serial.print("Current: ");
  Serial.println(currentA,6); //displays current through resistor A in mA

  delay(dutyCycle*period);

  digitalWrite(mosfetgate, LOW); // turn the MOSFET on
  voltageA1 = adc.newAnalogRead(resistorA1Pin)*inputvoltage/adc.getMaxPossibleReading(); //voltage at the front of resistor A
  voltageA2 = adc.newAnalogRead(resistorA2Pin)*inputvoltage/adc.getMaxPossibleReading(); //voltage at the back of resistor A
  currentA = (voltageA1 - voltageA2)/resistorA;
  Serial.print("Current: ");
  Serial.println(currentA,6); //displays current through resistor A in mA

  delay(1 - dutyCycle*period);



  //take a reading on the analog pin
  //note: this is an overloaded function, so there are 4 ways to do it!
  //Method 1: just take a reading, providing the pin number only; the library will use other values as necessary 
  //  that you already set with the "adc.set..." functions above
  // analogReading = adc.newAnalogRead(pin); //get the avg. of [num_samples] [bits_of_resolution]-bit readings 
  //Method 2: update what the library has stored for bitsOfResolution, AND take a new reading
//  analogReading = adc.newAnalogRead(pin,bitsOfResolution); //UNCOMMENT TO USE
  //Method 3: update what the library has stored for bitsOfResolution and numSamplesToAvg, AND take a new reading
//  analogReading = adc.newAnalogRead(pin,bitsOfResolution,numSamplesToAvg); //UNCOMMENT TO USE
  //Method 4: update what the library has stored for bitsOfResolution and numSamplesToAvg, then set the ADCSpeed, AND take a new reading
//  analogReading = adc.newAnalogRead(pin,bitsOfResolution,numSamplesToAvg,ADCSpeed); //UNCOMMENT TO USE
  


  // calculate the current
  //float currentA = voltageA1 - voltageA2;
  //float currentB = voltageB1;

  // // read the system output
  // output = currentA; // replace with your own function to read the system output
  // // calculate the error signal
  // error = setpoint - output;
  // // calculate the integral term
  // integral += error;
  // // calculate the derivative term
  // derivative = error - prevError;
  // // calculate the input to the system using the PID controller
  // input = Kp * error + Ki * integral + Kd * derivative;
  // // update the previous error signal
  // prevError = error;
  // // send the input signal to the system
  // sendInput(input); // replace with your own function to send the input signal to the system
  // // print the PID controller variables to the serial monitor
  // Serial.print("Error: ");
  // Serial.print(error);
  // Serial.print(", Input: ");
  // Serial.println(input);

  // float analog_reading = adc.analogReadXXbit(resistorA1Pin, bits_of_precision, num_samples);
  // Serial.print("MOSFET voltage ON: "); 
  // Serial.println(analog_reading*5/65472.0);

  // digitalWrite(mosfetgate, HIGH); // turn the MOSFET on
  // float voltageA1 = analogRead(resistorA1Pin)*inputvoltage/1024;
  // float voltageA2 = analogRead(resistorA2Pin)*inputvoltage/1024;
  // float voltageB1 = analogRead(resistorB1Pin)*inputvoltage/1024;
  // float voltageM = analogRead(mosfetPin)*inputvoltage/1024;
  // Serial.print("MOSFET voltage ON: ");
  // Serial.println(voltageM + p1 + voltageA1 + p1 + voltageA2 + p1 + voltageB1);
  // delay(1000); // duty cycle where it is on
  // digitalWrite(mosfetgate, LOW); // turn the MOSFET on
  // voltageA1 = analogRead(resistorA1Pin)*inputvoltage/1024;
  // voltageA2 = analogRead(resistorA2Pin)*inputvoltage/1024;
  // voltageB1 = analogRead(resistorB1Pin)*inputvoltage/1024;
  // voltageM = analogRead(mosfetPin)*inputvoltage/1024;
  // Serial.print("MOSFET voltage OFF: ");
  // Serial.println(voltageM + p1 + voltageA1 + p1 + voltageA2 + p1 + voltageB1);
  // delay(1000); // duty cycle where it is on
  
}
