//  The circuit:
//  * VSS pin to ground
//  * VDD pin to 5V
//  * V0 pin to resistor then to ground
//  * LCD RS pin to digital pin 2
//  * LCD R/W pin to ground
//  * LCD Enable pin to digital pin 3
//  * LCD D4 pin to digital pin 9
//  * LCD D5 pin to digital pin 10
//  * LCD D6 pin to digital pin 11
//  * LCD D7 pin to digital pin 12
#include<FastLED.h>
#include <LiquidCrystal.h>
#define NUM_LEDS 24
#define DATA_PIN 7
#define CLOCK_PIN 8
CRGBArray<NUM_LEDS> leds;
LiquidCrystal lcd(2, 3, 9, 10, 11, 12);
int sensorPin = 0; 
float timeSeconds = 600;
float timeMilliSec = timeSeconds*1000;

void setup()
{ 
  lcd.begin(16, 2); 
  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR, DATA_RATE_MHZ(12)>(leds, NUM_LEDS);
  Serial.begin(9600);
}
 
void loop()
{
  lcd.clear();
  lcd.setCursor(0,0);
  //How much time has past
  lcd.print(millis()/1000);
  lcd.print(" s past.");
  //How much time is left
  int timeLeft = timeSeconds - (millis()/1000);
  lcd.setCursor(0,1);
  lcd.print(timeLeft);
  lcd.print(" s left.");

//How many are red or green
  float fraction = millis()/timeMilliSec;
  float numbRed = (1 - fraction)*NUM_LEDS;
  int numbLitGreen = round(NUM_LEDS - numbRed);
  int numbLitRed = NUM_LEDS - numbLitGreen ;

  /////Fix voltage difference for different LEDs causeing different brightness
  for(int i=0; i<numbLitGreen; i++){
    leds[i].setRGB(0, 125, 0);//green
    delay(100);
    FastLED.show();
  }
  for(int i=numbLitGreen; i<numbLitRed; i++){
    leds[i].setRGB( 125, 0, 0);//red
    delay(100);
    FastLED.show();
  }
  delay(800);
}