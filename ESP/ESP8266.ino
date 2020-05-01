
/*
    This sketch demonstrates how to scan WiFi networks.
    The API is almost the same as with the WiFi Shield library,
    the most obvious difference being the different file you need to include:
*/
#include "ESP8266WiFi.h"

void setup() {
  Serial.begin(9600);

  // Set WiFi to station mode and disconnect from an AP if it was previously connected
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  //Serial.println("Setup done");
}

void loop() {
  //Serial.println("scan start");

  // WiFi.scanNetworks will return the number of networks found
  int n = WiFi.scanNetworks();
  //Serial.println("scan done");
  if (n == 0) {
    Serial.println("no networks found");
  } else {
    //Serial.print(n);
    //Serial.println(" networks found");
    for (int i = 0; i < n; ++i) {
      // Print SSID and RSSI for each network found
      //Serial.print(i + 1);
      //Serial.print(": ");
      
      String mac=WiFi.BSSIDstr(i);
      if(mac!="DE:4F:22:3B:10:3C")
      {
        String temp = WiFi.SSID(i);
        if(temp.startsWith("()"))
        {
          writeString(temp);
          writeString("\n");
        }
        
      }
      //Serial.print(" (");
      //Serial.print(WiFi.RSSI(i));
      //Serial.print(")");
      //Serial.println((WiFi.encryptionType(i) == ENC_TYPE_NONE) ? " " : "*");
      
      //delay(10);
    }
    writeString("($#,$#,$#,$#,$#,$#,$#,$#,$#,$#,");
  }
  Serial.println("");

  // Wait a bit before scanning again
  //delay(1000);
}
void writeString(String stringData) { // Used to serially push out a String with Serial.write()

  for (int i = 0; i < stringData.length(); i++)
  {
    Serial.write(stringData[i]);   // Push each char 1 by 1 on each loop pass
  }

}// end writeString
