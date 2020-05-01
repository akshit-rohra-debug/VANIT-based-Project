//Updated 7-6-2019


#include <SoftwareSerial.h>
#include <math.h>
#include <Wire.h>
#include <QMC5883L.h>
char getdirections[9][3] = {{'N'},{'N','E'},{'E'},{'S','E'},{'S'},{'S','W'},{'W'},{'N','W'},{'N'}};
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin
#include <SPI.h>          // f.k. for Arduino-1.5.2
#include "Adafruit_GFX.h"// Hardware-specific library
#include <MCUFRIEND_kbv.h>
#include<math.h>
MCUFRIEND_kbv tft;
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define RADIUS  10
int my_x, my_y, h, w;
int x[10]={0}, y[10]={0}, counter= -1;
double val = M_PI/180;
QMC5883L compass;
/*
 * ESP8266 14-15 (otherserial)
 * NodeMCU 18-17 (SelfSerial)
 * U1 25-27-29 (T-E-L)
 * U2 33-35-37
 * U3 46-48-50
 * U4 47-49-51
 * Buzzer 40
 */

double endlat1 = 19.075983, endlng1 = 72.877655; //Mumbai
double startlat2 = 18.520430, startlng2 = 73.856743; //Pune


double lat1,lng1;
double lat2,lng2;
 
const int pingPinF = 25; // Trigger Pin of Ultrasonic Sensor
const int echoPinF = 27; // Echo Pin of Ultrasonic Sensor

const int pingPinR = 46; // Trigger Pin of Ultrasonic Sensor
const int echoPinR = 48; // Echo Pin of Ultrasonic Sensor

const int pingPinB = 47; // Trigger Pin of Ultrasonic Sensor
const int echoPinB = 49; // Echo Pin of Ultrasonic Sensor

const int pingPinL = 33; // Trigger Pin of Ultrasonic Sensor
const int echoPinL = 35; // Echo Pin of Ultrasonic Sensor

const int LEDF = 29;
const int LEDB = 51;
const int LEDL = 37;
const int LEDR = 50;
const int BUZZ = 40;
int count=0;
void setup()
{
 Serial3.begin(9600);
 Serial1.begin(9600);
 Serial.begin(9600);
 Wire.begin();
 compass.init();
 pinMode(10,OUTPUT);
 pinMode(11,OUTPUT);
 pinMode(BUZZ,OUTPUT);
 pinMode(LEDF,OUTPUT);
 pinMode(LEDB,OUTPUT);
 pinMode(LEDL,OUTPUT);
 pinMode(LEDR,OUTPUT);

//Display setup
  uint16_t ID = tft.readID();
    if (ID == 0xD3D3) ID = 0x9481;
    tft.begin(ID);
    tft.fillScreen(BLACK);
    h = tft.height();
    w = tft.width();
    tft.setCursor((w-175)/2,(h-50)/2);
    tft.setTextSize(5);
    tft.println("Jaguar");
    tft.setTextSize(2);
    tft.println("         Inspired");
 delay(5000);

    tft.fillScreen(BLACK);
    tft.setCursor(85, 0);
    tft.setTextColor(WHITE);  tft.setTextSize(3);
    tft.println("!!!MAP!!!");
    int x1, x2, y1, y2;
    y1 = y1 + h/10;
    x1 = x1 + w/5;
    y2 = y1 + h - h/5;
    x2 = x1;
    tft.drawLine(x1-25,y1,x2-25,y2,WHITE);
    x1 = w-w/5;
    x2 = x1;
    tft.drawLine(x1+25,y1,x2+25,y2,WHITE);
    my_x = w/2;
    my_y = h/2;
    tft.fillCircle(my_x,my_y,RADIUS,BLUE);
}
void plotcar(int x, int y, uint16_t color) {
  tft.fillCircle(x,y,RADIUS,color);
  delay(10);
}
void vanishcar(int x, int y, uint16_t color)  {
  tft.fillCircle(x,y,RADIUS,color);
}


void makeHigh(int flag)
{
  if(flag==0)
  digitalWrite(LEDL, HIGH);
  else if(flag==1)
  digitalWrite(LEDR, HIGH);
  else if(flag==2)
  digitalWrite(LEDF, HIGH);
  else if(flag==3)
  digitalWrite(LEDB, HIGH);
  digitalWrite(BUZZ,HIGH);
  
}
void makeLow(int flag)
{
  if(flag==0)
  digitalWrite(LEDL, LOW);
  else if(flag==1)
  digitalWrite(LEDR, LOW);
  else if(flag==2)
  digitalWrite(LEDF, LOW);
  else if(flag==3)
  digitalWrite(LEDB, LOW);
  delay(500);
  digitalWrite(BUZZ,LOW);
}
bool checkThreshold(int cm,int flag)
{
  if(cm<10)
  {
    
    //delay(500);
    if(flag==0)
    makeHigh(0);
    else
    makeHigh(flag);
  }
  else
  {
    if(flag==0)
    makeLow(0);
    else
    makeLow(flag);    
  }
}
double toRad(double val)
{
  return val*3.1452/180;
}
double fundist(double lat1,double lng1, double lat_val, double lng_val)
{
  long double R=6371000;
  double lat_dif=toRad(lat1-lat_val);
  double lng_dif=toRad(lng1-lng_val);
  lat1=toRad(lat1);
  lat_val=toRad(lat_val);
  double a=sin(lat_dif/2)*sin(lat_dif/2)+cos(lat_val)*cos(lat1)*sin(lng_dif/2)*sin(lng_dif/2);
  double c=2*asin(sqrt(a));
  double distance=R*c;
  return distance;
} 
void loop(){
 char seq;
 boolean StringReady = false;
 String latString;
 String lngString;
 
 if (Serial3.available())
 {
    String temp=Serial3.readStringUntil('(');
    temp = Serial3.readStringUntil(',');
    //Serial.println(temp);
    if(temp.startsWith(")("))
    {
      latString = temp.substring(2);
      lngString = Serial3.readStringUntil(')');
      lat1=latString.toFloat();
      lng1=lngString.toFloat(); 
      Serial.println("otherslocation: ");
      Serial.println(lat1,6);
      Serial.println(lng1,6);
      count=0;
      plotcar(50,50,GREEN);
    }
    else
    {
      count++;
      plotcar(50,50,GREEN);
      if(count>=3)
      {
        count=0;
        lat1=0;
        lng1=0;
      }
    }
  }
  else
    {
      count++;
      plotcar(50,50,RED);
      if(count>=3)
      {
        count=0;
        lat1=0;
        lng1=0;
      }
    }
  
  if(Serial1.available())
  {
    Serial.println("HELLO");
    seq = Serial1.read();
    while(seq!='(')
    {
      seq = Serial1.read();
    }
    if(seq=='(')
    {
      String temp = Serial1.readStringUntil(',');
      //Serial.println(temp);
      if(temp.startsWith(")("))
      {
        String latitude,longitude;
        latitude = temp.substring(2);
        longitude = Serial1.readStringUntil(')');
        lat2=latitude.toFloat();
        lng2=longitude.toFloat();
        Serial.println(lat2,6);
        Serial.println(lng2,6);
      }
    }
  }
  double distance;
  distance=fundist(lat1,lng1,lat2,lng2);
  Serial.println("Distance="+String(distance)+"meters");

//Direction in degrees of the car
  int x1,y1,z1;
  compass.read(&x1,&y1,&z1);
  float heading = atan2(y1, x1);
  float declinationAngle = 0.0037;
  heading += declinationAngle;
  if(heading < 0)
    heading += 2*PI;
  if(heading > 2*PI)
    heading -= 2*PI;
  float headingDegrees = heading * 180/M_PI; 

  Serial.print("x: ");
  Serial.print(x1);
  Serial.print("    y: ");
  Serial.print(y1);
  Serial.print("    z: ");
  Serial.print(z1);
  Serial.print("    heading: ");
  Serial.print(heading);
  Serial.print("    Radius: ");
  Serial.print(headingDegrees);
  Serial.println();

//Direction of the incoming vehicle
  double radians;
  startlng2 = lng2;
  startlat2 = lat2;
  endlng1 = lng1;
  endlat1 = lat1;
  double y2 = endlng1-startlng2;
  double x2 = endlat1-startlat2;
  
  radians = atan2(y2,x2);
  double compassreading = radians * (180/M_PI);
  int coorindex = round(compassreading/45);
  if(coorindex<0)
  {
    coorindex += 8;
  }
  Serial.print("Compass Reading: ");
  Serial.print(compassreading);
  Serial.print(" - Direction: ");
  Serial.print(getdirections[coorindex]);
  Serial.println();
  
//Display code
  //distance in distance variable
  double d, degree = compassreading, phi = headingDegrees;
  //scaling distance for the map
    if(distance<100)
      d = 20+(distance/4);
    else
      d = 70+(distance/10);
  int i=0;
  if(counter!=-1)
      {
        for(i=0;i<=counter;i++)
        {
          vanishcar(x[i],y[i],BLACK);
        }
        counter = -1;
      }
      tft.fillCircle(my_x,my_y,RADIUS,BLUE);
  if(lat1!=0 || lng1!=0)
  {
    Serial.print("D:");
    Serial.println(d);
    
    
      if(phi!=0 || phi!=360)
      {
        if(phi<=180)
        {
          degree = degree+phi;
        }
        else  //between 180 & 360
        {
          phi = 360 - phi;
          degree = degree - phi;
        }
      }
      if(degree<-180)
      {
          degree = 360 + degree;  
      }
      else if(degree>180)
      {
          degree = degree - 360;  
      }
      if((int)degree==0)
      {
        counter++;
        x[counter] = my_x;
        y[counter] = my_y - d;
        plotcar(x[counter],y[counter],RED);
      }
      else if(degree>0 && degree<90)
      {
        counter++;
        x[counter] = my_x + d*sin(degree*val);
        y[counter] = my_y - d*cos(degree*val);
        plotcar(x[counter],y[counter],RED);
      }
      else if((int)degree==90)
      {
        counter++;
        x[counter] = my_x + d;
        y[counter] = my_y;
        plotcar(x[counter],y[counter],RED);
      }
      else if(degree>90 && degree<180)
      {
        counter++;
        x[counter] = my_x + abs(d*cos((degree-90)*val));
        y[counter] = my_y + abs(d*sin((degree-90)*val));
        plotcar(x[counter],y[counter],RED);
      }
      else if((int)degree==180)
      {
        counter++;
        x[counter] = my_x;
        y[counter] = my_y + d;
        plotcar(x[counter],y[counter],RED);
      }
      else if(degree<0 && degree>-90)
      {
        degree = degree * -1;
        counter++;
        x[counter] = my_x - d*sin(degree*val);
        y[counter] = my_y - d*cos(degree*val);
        plotcar(x[counter],y[counter],RED);
      }
      else if((int)degree==-90)
      {
        counter++;
        x[counter] = my_x - d;
        y[counter] = my_y;
        plotcar(x[counter],y[counter],RED);
      }
      else if(degree<-90 && degree>-180)
      {
        degree = degree*-1;
        counter++;
        x[counter] = my_x - abs(d*cos((degree-90)*val));
        y[counter] = my_y + abs(d*sin((degree-90)*val));
        plotcar(x[counter],y[counter],RED);
      }
      else if((int)degree==-180)
      {
        counter++;
        x[counter] = my_x;
        y[counter] = my_y + d;
        plotcar(x[counter],y[counter],RED);
      }
      delay(500);
  }

    //ULTRASONIC
    long durationL, inchesL, cmL;
    long durationR, inchesR, cmR;
    long durationF, inchesF, cmF;
    long durationB, inchesB, cmB;
    pinMode(pingPinL, OUTPUT);
    digitalWrite(pingPinL, LOW);
    digitalWrite(pingPinL, HIGH);
    digitalWrite(pingPinL, LOW);
    pinMode(echoPinL, INPUT);
    durationL = pulseIn(echoPinL, HIGH);
    cmL = microsecondsToCentimeters(durationL);
    Serial.print(cmL);
    Serial.print("cmL");
    Serial.println();
    checkThreshold(cmL,0);
    
    pinMode(pingPinR, OUTPUT);
    digitalWrite(pingPinR, LOW);
    digitalWrite(pingPinR, HIGH);
    digitalWrite(pingPinR, LOW);
    pinMode(echoPinR, INPUT);
    durationR = pulseIn(echoPinR, HIGH);
    cmR = microsecondsToCentimeters(durationR);
    Serial.print(cmR);
    Serial.print("cmR");
    Serial.println();
    checkThreshold(cmR,1);

    pinMode(pingPinF, OUTPUT);
    digitalWrite(pingPinF, LOW);
    digitalWrite(pingPinF, HIGH);
    digitalWrite(pingPinF, LOW);
    pinMode(echoPinF, INPUT);
    durationF = pulseIn(echoPinF, HIGH);
    cmF = microsecondsToCentimeters(durationF);
    Serial.print(cmF);
    Serial.print("cmF");
    Serial.println();
    checkThreshold(cmF,2);
    
    pinMode(pingPinB, OUTPUT);
    digitalWrite(pingPinB, LOW);
    digitalWrite(pingPinB, HIGH);
    digitalWrite(pingPinB, LOW);
    pinMode(echoPinB, INPUT);
    durationB = pulseIn(echoPinB, HIGH);
    cmB = microsecondsToCentimeters(durationB);
    Serial.print(cmB);
    Serial.print("cmB");
    Serial.println();
    checkThreshold(cmB,3);
    

 }
long microsecondsToInches(long microseconds) {
   return microseconds / 74 / 2;
}

long microsecondsToCentimeters(long microseconds) {
   return microseconds / 29 / 2;
}
