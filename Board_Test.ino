#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h> // for altimeter
#include <Adafruit_FRAM_I2C.h> // for FRAM

#define beepPin 4
#define redLEDPin 5
#define greenLEDPin 6
#define relay1Pin 7
#define relay2Pin 8

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085); // for altimeter
Adafruit_FRAM_I2C fram = Adafruit_FRAM_I2C(); // for FRAM

// uint16_t framAddr = 0; // check to see if this is needed
sensor_t sensor;

void setup() 
{
  Serial.begin(9600); //set up serial monitor
  
// configure effector pins for output
  pinMode(beepPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  
// now for the tests
  delay(4000); // pause 2 seconds for user to get serial monitor
  Serial.println("FLIGHT COMPUTER TEST"); Serial.println();
  delay(2000);
  Serial.println("Testing buzzer"); effectorTest(beepPin);
  Serial.println("Testing red LED"); effectorTest(redLEDPin);
  Serial.println("Testing green LED"); effectorTest(greenLEDPin);
  Serial.println("Testing relay 1"); effectorTest(relay1Pin);
  Serial.println("Testing relay 2"); effectorTest(relay2Pin);
  Serial.print("Testing pressure sensor...");
  bmp.getSensor(&sensor); // attaching the pressure sensor
  if (bmp.begin()) {Serial.println(" good.");}
  else {Serial.println(" bad.");}
  Serial.print("Testing FRAM...");
  if (fram.begin()) {Serial.println(" good.");}
  else {Serial.println(" bad.");}
}

void effectorTest(int pinNo)
{
  digitalWrite(pinNo, HIGH);
  delay(2000);
  digitalWrite(pinNo, LOW);
}

void loop() 
{
  if (digitalRead(relay1Pin)) Serial.println("HIGH");
  else Serial.println("LOW");
  delay(200);
}
