#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "C:\Users\nchen\Documents\PlatformIO\Projects\GPS_i2c_Slave\.pio\libdeps\nanoatmega328\TinyGPSPlus\src\TinyGPS++.h"

int heading(double lat1, double lon1, double lat2, double lon2);
int distance(double lat1, double lon1, double lat2, double lon2);

#define TX_PIN 2
#define RX_PIN 3
#define R 5
#define G 8
#define B 7
#define WAYPOINT_THRESHOLD 2
#define GPSBAUD 9600
#define DEV_NUM 7

struct Data {
  double heading;
  double distance;
};

// ----- GPS
const double coordinates[][2] = {{37.3250377, -121.9821168}, {37.325046, -121.981990}, {37.3250377, -121.9821168}};
int target = 0;


TinyGPSPlus gps;
SoftwareSerial gpsSerial(TX_PIN, RX_PIN);
SoftwareSerial comSerial(10, 9);

int dist, h = 0;
int thetaScaled;



void setup() {
  pinMode(R, OUTPUT);
  pinMode(G, OUTPUT);
  pinMode(B, OUTPUT);

  Serial.begin(9600);
  gpsSerial.begin(GPSBAUD);
  
  // CHECK NUM SATELLITES
  while(gps.satellites.value() < 5) {
    while(gpsSerial.available() > 0){
      gps.encode(gpsSerial.read());
    }
    Serial.println(gps.satellites.value());
    digitalWrite(R, HIGH);
    delay(100);
    digitalWrite(R, LOW);
    while(gpsSerial.available() > 0){
      gps.encode(gpsSerial.read());
    }
    Serial.println(gps.satellites.value());
    digitalWrite(G, HIGH);
    delay(100);
    digitalWrite(G, LOW);
    while(gpsSerial.available() > 0){
      gps.encode(gpsSerial.read());
    }
    Serial.println(gps.satellites.value());
    digitalWrite(B, HIGH);
    delay(100);
    digitalWrite(B, LOW);
  }

comSerial.begin(9600);

}

boolean gpsFlag = false;

void loop() {
  gpsSerial.listen();

  //MAIN PROGRAM HERE
  while(gpsSerial.available() > 0){
    gps.encode(gpsSerial.read());
    gpsFlag = true;
  }

  if(gpsFlag /*&& gps.location.isValid()*/ && target != sizeof(coordinates)){
    dist = distance(coordinates[target][0], coordinates[target][1], gps.location.lat(), gps.location.lng());
    h = heading(coordinates[target][0], coordinates[target][1], gps.location.lat(), gps.location.lng());

    Serial.print("Distance to Target: " + String(dist) + "\tDesired Heading: " + String(h) + "\tupdated");


    if(dist < WAYPOINT_THRESHOLD && dist >= 0){
      digitalWrite(B, LOW);
      digitalWrite(G, LOW);
      digitalWrite(R, LOW);
      
      Serial.println("\n\n");
      Serial.println("\tWAYPOINT " + String(target) + " LOCATED...");
      Serial.println("-------------------------------\n\n");

      //WAYPOINT LOCATED      
      target+=1;  //increment to next waypoint

      if(target == sizeof(coordinates)/8){
        // if Last location, infinite green LED blink
        digitalWrite(B, LOW);
        digitalWrite(G, LOW);
        digitalWrite(R, LOW);
        while(true) {
          digitalWrite(G, HIGH);
          delay(50);
          digitalWrite(G, LOW);
          delay(50);
        }
      }
      
      for(int i = 0; i < 20; i++){
        digitalWrite(G, HIGH);
        delay(100);
        digitalWrite(G, LOW);
        delay(100);
      } 
    }
  }else{
    Serial.print("Distance to Target: " + String(dist) + "\tDesired Heading: " + String(h));
  }

  comSerial.print(String(h) + '\t' + String(dist) + '\n');  // send GPS computation to navigation unit
  //delay(20);

  Serial.println();
  gpsFlag = false;

}

int distance(double lat1, double lon1, double lat2, double lon2)
{
  // Conversion factor from degrees to radians (pi/180)
  const double toRadian = 0.01745329251;

  // First coordinate (Radians)
  double lat1_r = lat1 * toRadian;
  //double lon1_r = lon1 * toRadian;

  // Second coordinate (Radians)
  double lat2_r = lat2 * toRadian;  
  //double lon2_r = lon2 * toRadian;

  // Delta coordinates 
  double deltaLat_r = (lat2 - lat1) * toRadian;
  double deltaLon_r = (lon2 - lon1) * toRadian;

  // Distance
  double a = sin(deltaLat_r/2)*sin(deltaLat_r/2) + cos(lat1_r) * cos(lat2_r) * sin(deltaLon_r/2) * sin(deltaLon_r/2);
  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  double distance = 6371 * c * 1000;

  return int(distance);
}

int heading(double lat1, double lon1, double lat2, double lon2)
{
  // Conversion factor from degrees to radians (pi/180)
  const double toRadian = 0.01745329251;
  // Conversion factor from degrees to radians (180/pi)
  const double toDegrees = 57.2957795131;
  
  double x, y;
  double latCurrent = lat1 * toRadian;
  double lonCurrent = lon1 * toRadian;
  double latDest = lat2 * toRadian;
  double lonDest = lon2 * toRadian;
  
  x = cos(latDest) * sin(lonDest - lonCurrent);
  y = cos(latCurrent) * sin(latDest) - sin(latCurrent) * cos(latDest) * cos(lonDest - lonCurrent);
  
  double val = 180 + (atan2(x, y)*toDegrees);
  if(val > 359){
    return 360 - val;
  }
  return int(val);
}