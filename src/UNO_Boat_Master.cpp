#include <Arduino.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <RH_RF95.h>
#include <AltSoftSerial.h>

//Initialisering for I2C og 2.4Ghz
int ch1; // Here's where we'll keep our channel values
int ch2;
int ch3;
int ch4;

const int numValues = 3;
int values[numValues];

int mappedch1;
int mappedch2;
int mappedch3;

unsigned long StartTime = 0;
unsigned long currentTime = 0;
bool Pulsing = true;
bool Switch = false;

const float alpha = 0.2; // Smoothing factor (adjust as needed)
float smoothedCh3 = 0; // Variable to store smoothed value of ch3


//Initialisering for gps og rf
// Define the pins for AltSoftSerial
#define GPS_RX_PIN 8
#define GPS_TX_PIN 7

AltSoftSerial gpsSerial; // Define AltSoftSerial object for GPS communication

TinyGPSPlus gps; // Create a TinyGPS++ object

// Define LoRa pins
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2

// Define LoRa frequency
#define RF95_FREQ 915.0

// Singleton instance of the radio driver'
RH_RF95 rf95(RFM95_CS, RFM95_INT);


void setup() {
  Serial.begin(9600);

  //SETUP TIL I2C OG 2.4Ghz
  pinMode(3, INPUT); // Set our input pins as such
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(A0, INPUT);
  Wire.begin();// Starter I2C Bus as Master 


  //SETUP TIL GPS OG RF
  gpsSerial.begin(9600);// Initialize AltSoftSerial for GPS module
  // Set LoRa reset pin as output
    pinMode(RFM95_RST, OUTPUT); 
  // Set LoRa reset pin high
  digitalWrite(RFM95_RST, HIGH);
  // Initialize LoRa module
    if (!rf95.init()) {
        Serial.println("LoRa radio init failed");
        while (1);
    }
    Serial.println("LoRa radio init OK!");
    // Set LoRa frequency
    if (!rf95.setFrequency(RF95_FREQ)) {
        Serial.println("setFrequency failed");
        while (1);
    }
    Serial.print("Set Freq to: ");
    Serial.println(RF95_FREQ);
    // Set LoRa transmit power
    rf95.setTxPower(23, false);
}

void sendValues(int * values, int numValues){
  Wire.beginTransmission(9);
  for (int i = 0; i < numValues; i++) {
    Wire.write((uint8_t*)&values[i], sizeof(int)); // Send each value individually
  }
  Wire.endTransmission();

}

void RFMessage(){
// Read data from GPS module
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            if (gps.location.isValid()) {
                // Prepare GPS data to send over LoRa
                String gpsData = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) + "," + String(Pulsing);
                
                // Prepend GPS data with a header to indicate it is GPS data
                gpsData = "GPS:" + gpsData;
                
                // Convert GPS data to char array
                char radiopacket[32];
                gpsData.toCharArray(radiopacket, 32);
                
                // Send GPS data over LoRa
                Serial.println("Sending GPS data over LoRa");
                rf95.send((uint8_t *)radiopacket, gpsData.length());
                
                // Wait for LoRa transmission to complete
                rf95.waitPacketSent();
            }
        }
    }

}

void readPWM(){
  ch1 = pulseIn(3, HIGH, 25000); // Read the pulse width of 
  ch2 = pulseIn(5, HIGH, 25000); // each channel
  ch3 = pulseIn(6, HIGH, 25000);
  ch4 = analogRead(A0);

  if((ch4 > 300) && !Switch){
   StartTime = millis();
   Switch = true;
  }

  if((ch4 < 200)){
    Pulsing = true;
    Switch = false;
  }

  if(Switch){
    currentTime = millis();
    if((currentTime - StartTime) > 500){
      Pulsing = false;
  }
  }

  if(Pulsing){
    mappedch1 = 0;
    mappedch2 = 0;
    mappedch3 = 0;
  }else{

    mappedch2 = constrain(map(ch2, 2010, 1075, -100, 100), -100, 100);
  //mappedch3 = constrain(map(ch3, 1469, 1575, -100, 100), -100, 100);
  smoothedCh3 = alpha * ch3 + (1 - alpha) * smoothedCh3;
  mappedch3 = constrain(map(smoothedCh3, 1475, 1570, -100, 100), -100, 100);

  if(ch1 > 1800){
    mappedch1 = 2;
  }
  else{
    mappedch1 = 1;
  }

  if( mappedch2 < 5 && mappedch2 > -5){
    mappedch2 = 0;
  }

  if( mappedch3 < 8 && mappedch3 > -8){
    mappedch3 = 0;
  }

  }

  values[0] = mappedch1;
  values[1] = mappedch2;
  values[2] = mappedch3;

/*
  Serial.print("Switch state = ");
  Serial.print(values[0]);
  Serial.print("  ");
  Serial.print("Throttle value = ");
  Serial.print(values[1]);
  Serial.print("  ");
  Serial.print("Steering value = ");
  Serial.println(values[2]);
  Serial.println(ch3);
*/

}


void loop() {
  readPWM();
  sendValues(values, numValues);
  RFMessage();
  delay(50);
}


