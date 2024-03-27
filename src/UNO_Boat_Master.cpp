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

//Global state
int State = 0;

// Backup Connection vars
int BackupconState = 0;
int ReadCounter = 0;
int FilterValue = 0;
unsigned long readingInterval = 100;

const int numValues = 3;
int values[numValues];

int mappedch1;
int mappedch2;
int mappedch3;

int throttle;
int brake;
int RSSI;

unsigned long StartTime = 0;
unsigned long currentTime = 0;
bool Pulsing = true;
bool Switch = false;
bool backupConnectionState = 0;

const float alpha = 0.2; // Smoothing factor (adjust as needed)
float smoothedCh3 = 0; // Variable to store smoothed value of ch3


//Initialisering for gps og rf
bool RfConnection = false;
int failedconnections = 0;
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

int filterReadAvg(int pin, int interval){
  int value = 0;
  for(int i = 0; i<interval; i++){
    value += pulseIn(pin, HIGH, 25000);
  }
  return int(value/interval);
}

void filterRead(){
FilterValue += analogRead(A0);
ReadCounter ++;
if (ReadCounter > 20){
  if(FilterValue > 6000){
    backupConnectionState = true;
  } else {
    backupConnectionState = false;
  }
  ReadCounter = 0;
  FilterValue = 0;
}


}

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
  bool messageReceived = false;
// Read data from GPS module
    while (gpsSerial.available() > 0) {
        if (gps.encode(gpsSerial.read())) {
            if (gps.location.isValid()) {
                // Prepare GPS data to send over LoRa
                String gpsData = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
                
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
                messageReceived = true;

                if(messageReceived){
                uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
                uint8_t len = sizeof(buf);
                
              
                if (rf95.recv(buf, &len)){
                  String receivedMessage;
                  for (int i = 0; i < len; i++) {
                  receivedMessage += (char)buf[i];
                }

                if (receivedMessage.startsWith("C_Data:")) {
                 messageReceived = false;
                 // Extract GPS data from the message
                 String controllData = receivedMessage.substring(6); // Remove "C_Data" prefix
                 RSSI = rf95.lastRssi();
                 failedconnections = 0;

                 int commaIndex = controllData.indexOf(",");
                 if (commaIndex != -1){
                 throttle = controllData.substring(0, commaIndex).toInt();
                 brake = controllData.substring(commaIndex + 1).toInt();
                }
                 Serial.println(rf95.lastRssi(), DEC);
                 Serial.println(throttle);
                 Serial.println(brake);
                }else{
                 failedconnections ++;
                }
                }

                if(failedconnections > 5){
                  RfConnection = false;
                }else{
                  RfConnection = true;
                }
                
            }
        }
    }

}
}

void checkBackupConnection(){

  switch (BackupconState)
  {
  case 0 /* constant-expression */:
  Serial.println("State: 0");
    if(analogRead(A0) > 200){
      StartTime = millis();
      BackupconState = 1;
    } else {
      BackupconState = 20;
    }
  
  case 1:
  Serial.println("State: 1");
  if (analogRead(A0) < 200){
    BackupconState = 20;
  }
      if(millis() > StartTime+600){
        backupConnectionState = true;
      } else {
        break;
      }
  case 20:
  Serial.println("State: 20");
    backupConnectionState = false;
    BackupconState = 0;
    break;
  }

}


void readPWM(){
  ch1 = pulseIn(3, HIGH, 25000); // Read the pulse width of 
  ch2 = pulseIn(5, HIGH, 25000); // each channel
  ch3 = pulseIn(6, HIGH, 25000);

/*
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


  values[0] = mappedch1;
  values[1] = mappedch2;
  values[2] = mappedch3;
*/
}


void loop() {

  RFMessage();
  if(RfConnection){
    Serial.println("Sending land data to mega");
    readPWM();
  }else{
    readPWM();
    sendValues(values, numValues);
    Serial.println("Sending backup data to mega");
  }

// Serial.println(backupConnectionState);

}


