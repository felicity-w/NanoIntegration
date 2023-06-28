/*
  Arduino Constant Current


  Parts required:
  - two 2.2k resistors
  - two 820 resistors
  - one 10k potentiometer
  - three 1k resistor
  - one 1M resistor
  - one LED
  - 18V battery
  - one p-type MOSFET
  - one n-type MOSFET
  - one 100 mH inductor
  - one 470 uF capacitor
  - one diode
  - one Arduino Uno

*/

const int LED = 4;  // connected to pin 8 for the LED circuit
const int mosfetgate = 13;  // connected to pin 13 for the gate of the N-type mosfet
int volt_1000 = A1; // Input for the voltage detection for the current detector is in pin A1
int volt_buck = A0; // Input for the voltage after the buck circuit is connected to pin A0

float raw_volt_1000; // Gives the voltage (in bits) above the 1k resistor
float raw_volt_buck; // Gives the voltage (in bits) after the buck circuit
float act_volt_1000; // Gives the actual voltage (in volts) above the 1k resistor
float act_volt_buck; // Gives the actual voltage (in volts) after the buck circuit
float volt_circuit; // Gives the actual voltage across the GaN circuit
float current_1000; // Initialise the current across the 1k resistor
float rail_voltage = 18.3; // Input the actual rail voltage as checked by an external voltmeter
float point = 1024.0; // Intialise how many bits the analogue pins are able to resolve
float current_setpoint = 300; // In microamps, can be in range 150 uA to 800 uA

float previousReadTime = 0; // setting the timing of reading the voltage and current pins
float previousWriteTime = 0;  // setting the timing of writing the MOSFET and LED pins
float mosfetOnTime = 100;   // LED on time in microseconds (set as constant)
float mosfetOffTime = 54664 * pow(2.71828, -0.005 * current_setpoint) + 0;  // LED off time in microseconds 
bool mosfetState = LOW;
bool LEDState = LOW;
bool mosfet1000State = HIGH;

// Constants
const float Kp = 1.0;  // Proportional gain
const float Ki = 0.5;  // Integral gain
const float Kd = 0.2;  // Derivative gain

// PID controller Variables
float time_setpoint = 0;
float time_input = 0;
float output = 0;
float error = 0.0;      // Error (difference between setpoint and input)
float lastError = 0.0;  // Previous error
float proportional = 0.0; // Gain term 
float integral = 0.0;   // Integral term (accumulated error)
float derivative = 0.0; // Derivative term (rate of change of error)

void setup() {
  // initialize the inputs and outputs
  pinMode(LED, OUTPUT);
  pinMode(mosfetgate, OUTPUT);
  pinMode(volt_1000, INPUT);
  pinMode(volt_buck, INPUT);
  // initialize serial communication at 2000000 bits per second: (To reduce dead time in the code)
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
    raw_volt_buck = (float)analogRead(volt_buck);
    act_volt_buck = (raw_volt_buck * rail_voltage) / point;
    current_1000 = act_volt_1000 / 1000;
    volt_circuit = act_volt_buck - act_volt_1000; 
    // printing the values seen
    Serial.print(volt_circuit * 100, 3); // Print the voltage of the circuit in 0.01 V (to match the scale of the current)
    Serial.print(",");
    Serial.println(current_1000 * 1000000, 3); // Print the current of the circuit in uA

    // PID controller
    time_setpoint = 54664 * pow(2.71828, -0.005 * current_setpoint) + 0;
    time_input = 54664 * pow(2.71828, -0.005 * current_1000 * 1000000) + 0;
    error = time_setpoint - time_input;
    proportional = Kp * error;
    integral += Ki * error;
    derivative = Kd * (error - lastError);
    output = proportional + integral + derivative;
    mosfetOffTime = output;
    lastError = error;
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

    }
  }
}
