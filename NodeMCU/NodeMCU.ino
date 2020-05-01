#include "TinyGPS++.h";
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>

SoftwareSerial GPS_SoftSerial(5, 4);/* (Rx, Tx) */

TinyGPSPlus gps;      

double temp_lat, temp_long;

volatile float minutes, seconds;
volatile int degree, secs, mins;

int n=0;
void setup() {
  Serial.begin(9600); /* Define baud rate for serial communication */
  GPS_SoftSerial.begin(9600); /* Define baud rate for software serial communication */
  
  WiFi.mode(WIFI_AP);
  IPAddress myIP = WiFi.softAPIP();
  WiFi.persistent(false);
}

void loop() {
        smartDelay(1000); /* Generate precise delay of 1ms */
        unsigned long start;
        double lat_val, lng_val, alt_m_val;
        uint8_t hr_val, min_val, sec_val;
        bool loc_valid, alt_valid, time_valid;
        lat_val = gps.location.lat(); /* Get latitude data */
        loc_valid = gps.location.isValid(); /* Check if valid location data is available */
        lng_val = gps.location.lng(); /* Get longtitude data */
        alt_m_val = gps.altitude.meters();  /* Get altitude data in meters */
        alt_valid = gps.altitude.isValid(); /* Check if valid altitude data is available */
        hr_val = gps.time.hour(); /* Get hour */
        min_val = gps.time.minute();  /* Get minutes */
        sec_val = gps.time.second();  /* Get seconds */
        time_valid = gps.time.isValid();  /* Check if valid time data is available */
        if (!loc_valid)
        {          
          Serial.print("Latitude : ");
          Serial.println("*****");
          Serial.print("Longitude : ");
          Serial.println("*****");
        }
        else
        {
          String myString=String(n);

          temp_lat = lat_val;
          temp_long = lng_val;
          String latString = String(temp_lat,6);
          String lngString = String(temp_long,6);
          
          String ssid1 = String("()("+latString+","+lngString+")");
          int len = ssid1.length();
          char copy[25];
          ssid1.toCharArray(copy, len+2);

          Serial.println("()("+latString+","+lngString+")");
          WiFi.softAP(copy);
          n+=1;
          delay(750);
          
        }
        if (!alt_valid)
        {
          Serial.print("Altitude : ");
          Serial.println("*****");
        }
        if (!time_valid)
        {
          Serial.print("Time : ");
          Serial.println("*****");
        }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (GPS_SoftSerial.available())  /* Encode data read from GPS while data is available on serial port */
      gps.encode(GPS_SoftSerial.read());

  } while (millis() - start < ms);
}

void DegMinSec( double tot_val)   /* Convert data in decimal degrees into degrees minutes seconds form */
{  
  degree = (int)tot_val;
  minutes = tot_val - degree;
  seconds = 60 * minutes;
  minutes = (int)seconds;
  mins = (int)minutes;
  seconds = seconds - minutes;
  seconds = 60 * seconds;
  secs = (int)seconds;
}
