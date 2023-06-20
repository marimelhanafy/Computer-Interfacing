//////////////////// MAX30102 ////////////////////
#include "MAX30105.h"   //MAX3010x library
#include "heartRate.h"  //Heart rate calculating algorithm
#include "ESP32Servo.h"
#include <Wire.h>
MAX30105 particleSensor;
int Tonepin = 4;
const byte RATE_SIZE = 10;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;  //Time at which the last beat occurred
long previousMillis = 0;   // variable to store the previous time
const long interval = 60000; // interval (1 min)
float beatsPerMinute;
int beatAvg;
double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
double SpO2 = 0;
double ESpO2 = 90.0;
double FSpO2 = 0.7;   //filter factor for estimated SpO2
double frate = 0.95;  //low pass filter for IR/red LED value to eliminate AC component 
int i = 0;
int Num = 30;
#define FINGER_ON 7000
#define MINIMUM_SPO2 90.0
//////////////////// SH1106 ////////////////////
#include <qrbits.h>
#include <qrcode.h>
#include <qrencode.h>
#include <SH1106.h>                             // Include the SH1106 library
#define OLED_ADDR 0x3C                          // OLED I2C address
#define OLED_SDA 4                              // OLED SDA pin
#define OLED_SCL 15                             // OLED SCL pin
SH1106 display(OLED_ADDR, OLED_SDA, OLED_SCL);  // Create an SH1106 display object
//////////////////// DHT11 ////////////////////
#include "DHT.h"
#define DHTPIN 18 //Digital pin connected to the DHT sensor
#define DHTTYPE DHT11 //type we're using! (DHT 11)
DHT dht(DHTPIN, DHTTYPE); //Initialize DHT sensor.
//////////////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);  //Start the Serial Monitor
  Serial.println("System Start");
  //////////////////// SH1106 ////////////////////
  display.init();                             // Initialize the display
  display.flipScreenVertically();             // Flip the display vertically
  display.setFont(ArialMT_Plain_10);          // Set the font
  display.setTextAlignment(TEXT_ALIGN_LEFT);  // Set the text alignment
  //////////////////// MAX30102 ////////////////////
  //Use default I2C port, 400kHz speed
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 did not work!");
    while (1)
      ;
  }
  //Set up the wanted parameters
  byte ledBrightness = 0x7F;
  byte sampleAverage = 4;
  byte ledMode = 2;
  int sampleRate = 800;
  int pulseWidth = 215;
  int adcRange = 16384;
  //Configure sensor with these settings
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  particleSensor.enableDIETEMPRDY();
  //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeRed(0x0A);
  //Turn off Green LED
  particleSensor.setPulseAmplitudeGreen(0);
  //////////////////// DHT11 ////////////////////
  dht.begin();  //ask the sensor to start working
}
//////////////////////////////////////////////////////////////////////////////
////
void loop() {
  long currentMillis = millis();  // get the current time
  if (currentMillis - previousMillis >= interval) {   // check if it's time to say "Hi"
    previousMillis = currentMillis;       // save the current time for the next iteration
    if(beatAvg<60 || beatAvg>100 || ESpO2<75 || ESpO2>100){
      display.clear();  // Clear the display
      display.drawString(10, 10, "Warning");
      display.drawString(10, 20, "!!!");      
      return;
    }
    display.display();  // Update the display
  }
  //Reading the IR value
  //(it will permit us to know if there's a finger on the sensor or not)
  long irValue = particleSensor.getIR();
  if (irValue > FINGER_ON) {
    display.clear();  // Clear the display
    display.drawString(10, 10, String(beatAvg));
    display.drawString(10, 20, "BPM");
    Serial.println("7");
    if (beatAvg > 30) {
      display.drawString(10, 30, String(ESpO2));
      display.drawString(10, 40, "%");
      Serial.println("6");
    } else {
      display.drawString(10, 30, "---- %");
      Serial.println("5");
    }
    display.display();  // Update the display
    if (checkForBeat(irValue) == true) {
      display.clear();  // Clear the display
      display.drawString(10, 10, String(beatAvg));
      display.drawString(10, 20, "BPM");
      Serial.println("4");
      if (beatAvg > 30) {
        display.drawString(10, 30, String(ESpO2));
        display.drawString(10, 40, "%");
        Serial.println("3");
      } else {
        display.drawString(10, 30, "---- %");
        Serial.println("2");
      }
      display.display();  // Update the display
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);
      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
    }
    uint32_t ir, red;
    double fred, fir;
    //Check the sensor, read up to 3 samples
    particleSensor.check();
    if (particleSensor.available()) {
      i++;
      red = particleSensor.getFIFOIR();
      ir = particleSensor.getFIFORed();
      fred = (double)red;  //double
      fir = (double)ir;    //double
      //average red level by low pass filter
      avered = avered * frate + (double)red * (1.0 - frate);
      //average IR level by low pass filter
      aveir = aveir * frate + (double)ir * (1.0 - frate);
      //square sum of alternate component of red level
      sumredrms += (fred - avered) * (fred - avered);
      //square sum of alternate component of IR level
      sumirrms += (fir - aveir) * (fir - aveir);
      if ((i % Num) == 0) {
        double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
        SpO2 = -23.3 * (R - 0.4) + 100;
        ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;     //low pass filter
        if (ESpO2 <= MINIMUM_SPO2) ESpO2 = MINIMUM_SPO2;  //indicator for finger detached 
        if (ESpO2 > 100) ESpO2 = 99.9;
        sumredrms = 0.0;
        sumirrms = 0.0;
        SpO2 = 0;
        i = 0;
      }
      particleSensor.nextSample();
    }
  } else {
    for (byte rx = 0; rx < RATE_SIZE; rx++) rates[rx] = 0;
    beatAvg = 0;
    rateSpot = 0;
    lastBeat = 0;
    avered = 0;
    aveir = 0;
    sumirrms = 0;
    sumredrms = 0;
    SpO2 = 0;
    ESpO2 = 90.0;
    Serial.println("1");
    display.clear();                           // Clear the display
    display.drawString(10, 10, "No Finger!");  //Finger Please
    display.display();                         // Update the display
  }
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("111");
    display.drawString(80, 20, "T Failed");
    display.display();  // Update the display
    return;             //exit early (to try again)
  }
  if (t>50){
    Serial.println("111");
    display.drawString(80, 10, "Change");
    display.drawString(80, 20, "Place");
    display.display();  // Update the display
    return;
  }
  display.drawString(80, 10, String(h));
  display.drawString(80, 20, "Humi");
  display.drawString(80, 30, String(t));
  display.drawString(80, 40, "Temp");
  display.display();  // Update the display
  Serial.println("999");  
}