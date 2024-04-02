// 
// MSE 2202 TCS34725 colour sensor example
// 
//  Language: Arduino (C++)
//  Target:   ESP32-S3
//  Author:   Michael Naish
//  Date:     2024 03 05 
//

#define PRINT_COLOUR                                  // uncomment to turn on output of colour sensor data

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_TCS34725.h"
#include <MSE2202_Lib.h>

// Function declarations
void doHeartbeat();

// Constants
const int cHeartbeatInterval = 75;                    // heartbeat update interval, in milliseconds
const int cSmartLED          = 21;                    // when DIP switch S1-4 is on, SMART LED is connected to GPIO21
const int cSmartLEDCount     = 1;                     // number of Smart LEDs in use
const int cSDA               = 47;                    // GPIO pin for I2C data
const int cSCL               = 48;                    // GPIO pin for I2C clock
const int cTCSLED            = 14;                    // GPIO pin for LED on TCS34725
const int cLEDSwitch         = 46;                    // DIP switch S1-2 controls LED on TCS32725    
//const long cCountsRev = 880;
const long cCountsRev = 1000;

const int cPWMRes = 8;                    // bit resolution for PWM
const int cMinPWM = 150;                  // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;  // PWM value for maximum speed

// Variables
boolean heartbeatState       = true;                  // state of heartbeat LED
unsigned long lastHeartbeat  = 0;                     // time of last heartbeat state change
unsigned long curMillis      = 0;                     // current time, in milliseconds
unsigned long prevMillis     = 0;                     // start time for delay cycle, in milliseconds
bool cFlag = false;
bool gFlag = false;
bool cyanFlag = false;
unsigned char driveSpeed;         // motor drive speed (0-255)
unsigned long currentMicros;          // current microsecond count
unsigned long previousMicros;         // last microsecond count
unsigned long timerCount3sec = 0;     // 3 second timer count in milliseconds
boolean timeUp3sec = false;           // 3 second timer elapsed flag
double target = 0;
bool isGreen = false;
bool waitFlag = false;

Motion wheel = Motion();
Encoders encoder = Encoders();

#define MOTOR_A 35
#define MOTOR_B 36

#define ENCODER_A 15      // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_B 16      // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define POT_R1 1

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(cSmartLEDCount, cSmartLED, NEO_RGB + NEO_KHZ800);

// Smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {0, 0, 0, 5, 15, 30, 45, 60, 75, 90, 105, 120, 135, 
                                       150, 135, 120, 105, 90, 75, 60, 45, 30, 15, 5, 0};

// TCS34725 colour sensor with 2.4 ms integration time and gain of 4
// see https://github.com/adafruit/Adafruit_TCS34725/blob/master/Adafruit_TCS34725.h for all possible values
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_4X);
bool tcsFlag = 0;                                     // TCS34725 flag: 1 = connected; 0 = not found

void setTarget(int dir);

void setup() {
  Serial.begin(115200);                               // Standard baud rate for ESP32 serial monitor

  // Set up SmartLED
  SmartLEDs.begin();                                  // initialize smart LEDs object
  SmartLEDs.clear();                                  // clear pixel
  SmartLEDs.setPixelColor(0, SmartLEDs.Color(0,0,0)); // set pixel colours to black (off)
  SmartLEDs.setBrightness(0);                         // set brightness [0-255]
  SmartLEDs.show();                                   // update LED

  Wire.setPins(cSDA, cSCL);                           // set I2C pins for TCS34725
  pinMode(cTCSLED, OUTPUT);                           // configure GPIO to control LED on TCS34725
  pinMode(cLEDSwitch, INPUT_PULLUP);                  // configure GPIO to set state of TCS34725 LED 

  // Connect to TCS34725 colour sensor
  if (tcs.begin()) {
    Serial.printf("Found TCS34725 colour sensor\n");
    tcsFlag = true;
  } 
  else {
    Serial.printf("No TCS34725 found ... check your connections\n");
    tcsFlag = false;
  }


  wheel.driveBegin("D1", MOTOR_A, MOTOR_B, 6, 7);  // set up motors as Drive 1
  encoder.Begin(ENCODER_A, ENCODER_B, &wheel.iLeftMotorRunning);
}

void loop() {
  int pot = 0;
  uint16_t r, g, b, c;                                // RGBC values from TCS34725
  
  digitalWrite(cTCSLED, !digitalRead(cLEDSwitch));    // turn on onboard LED if switch state is low (on position)
  if (tcsFlag) {                                      // if colour sensor initialized
    tcs.getRawData(&r, &g, &b, &c);                   // get raw RGBC values
#ifdef PRINT_COLOUR            
      //Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
    if((c>60)&&(c<94)){
      cFlag = true;
    }
    else
      cFlag = false;
    
    if((g>b) && (g>r)){
      gFlag = true;
    }
    else gFlag = false;

    if(b<r){
      cyanFlag = true;
    }
    else cyanFlag = false;

    if(gFlag && cFlag && cyanFlag){
      //Serial.printf("Green\n");
      isGreen = true;
    }
    else
      isGreen = false;
      Serial.printf("");
    #endif
  }
  pot = analogRead(POT_R1);
  driveSpeed = cMaxPWM/1.3;

  currentMicros = micros();                        // get current time in microseconds
  if ((currentMicros - previousMicros) >= 1000) {  // enter when 1 ms has elapsed
    previousMicros = currentMicros;                // record current time in microseconds

    // 3 second timer, counts 3000 milliseconds
    timerCount3sec = timerCount3sec + 1;  // increment 3 second timer count
    if (timerCount3sec > 1000) {          // if 3 seconds have elapsed
      timerCount3sec = 0;                 // reset 3 second timer count
      timeUp3sec = true;                  // indicate that 3 seconds have elapsed

      if(isGreen){
      setTarget(1);
      wheel.Forward("D1", driveSpeed, 0);
      waitFlag = true;
      }else{
        if(!waitFlag){
        setTarget(-1);
        wheel.Left("D1", driveSpeed, 0);
        }
      }

      Serial.printf("Target: %f     Timer: %ul    Encoder: ", target, timerCount3sec);
      encoder.getEncoderRawCount();
      Serial.print(encoder.lRawEncoderCount);
      Serial.print("\n");
      if(isGreen){
        Serial.println("Green");
        Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
      }
      else
        Serial.println("Not green");
        Serial.printf("R: %d, G: %d, B: %d, C %d\n", r, g, b, c);
    }
    
    encoder.getEncoderRawCount();
    if(waitFlag&&(encoder.lRawEncoderCount >= target))
    {
      wheel.Stop("D1");
      waitFlag = false;
    }
    if((!waitFlag)&&(encoder.lRawEncoderCount <= target))
    {
      wheel.Stop("D1");
    }
  }
  doHeartbeat();                                      // update heartbeat LED

}

// update heartbeat LED
void doHeartbeat() {
  curMillis = millis();                               // get the current time in milliseconds
  // check to see if elapsed time matches the heartbeat interval
  if ((curMillis - lastHeartbeat) > cHeartbeatInterval) {
    lastHeartbeat = curMillis;                        // update the heartbeat time for the next update
    LEDBrightnessIndex++;                             // shift to the next brightness level
    if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) { // if all defined levels have been used
      LEDBrightnessIndex = 0;                         // reset to starting brightness
    }
    SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]); // set brightness of heartbeat LED
    SmartLEDs.setPixelColor(0, SmartLEDs.Color(0, 250, 0)); // set pixel colours to green
    SmartLEDs.show();                                 // update LED
  }
}

void setTarget(int dir) {
    encoder.getEncoderRawCount();
    if (dir == 1) {
      target = encoder.lRawEncoderCount + (cCountsRev/4);
    }
    if (dir == -1) {
      target = encoder.lRawEncoderCount - (cCountsRev/4);
    }
}