#include <Arduino.h>
#include <Wire.h>

int ch1; // Here's where we'll keep our channel values
int ch2;
int ch3;

const int numValues = 3;
int values[numValues];

int mappedch1;
int mappedch2;
int mappedch3;

const float alpha = 0.2; // Smoothing factor (adjust as needed)
float smoothedCh3 = 0; // Variable to store smoothed value of ch3

void setup() {
  pinMode(3, INPUT); // Set our input pins as such
  pinMode(5, INPUT);
  pinMode(6, INPUT);

  Serial.begin(9600);
  // Starter I2C Bus as Master
  Wire.begin(); 
}

void sendValues(int * values, int numValues){
  Wire.beginTransmission(9);
  for (int i = 0; i < numValues; i++) {
    Wire.write((uint8_t*)&values[i], sizeof(int)); // Send each value individually
  }
  Wire.endTransmission();

}

void loop() {
  ch1 = pulseIn(3, HIGH, 25000); // Read the pulse width of 
  ch2 = pulseIn(5, HIGH, 25000); // each channel
  ch3 = pulseIn(6, HIGH, 25000);

  mappedch2 = constrain(map(ch2, 2010, 1075, -100, 100), -100, 100);
  //mappedch3 = constrain(map(ch3, 1469, 1575, -100, 100), -100, 100);
  smoothedCh3 = alpha * ch3 + (1 - alpha) * smoothedCh3;
  mappedch3 = constrain(map(smoothedCh3, 1475, 1570, -100, 100), -100, 100);

  if(ch1 > 1800){
    mappedch1 = 1;
  }
  else{
    mappedch1 = 0;
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

/*
  Serial.print("Switch state = ");
  Serial.print(values[0]);
  Serial.print("  ");
  Serial.print("Throttle value = ");
  Serial.print(values[1]);
  Serial.print("  ");
  Serial.print("Steering value = ");
  Serial.println(values[2]);
*/
  Serial.println(ch3);
  sendValues(values, numValues);


  delay(200);
}


