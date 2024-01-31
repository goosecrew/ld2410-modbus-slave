#include <WiFi.h>
#include <WiFiClient.h>

char ssid[6] = "q.O.p";
char password[13] = "ILoveYouBaby";

WiFiClient wifiClient;

void setupWiFi() {

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  char emptyMsg[1] = "";
  char pointMsg[2] = ".";
  Serial.println(emptyMsg);

    // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(pointMsg);
  }
  Serial.println(emptyMsg);
  char connectedToMsg[14] = "Connected to ";
  Serial.println(connectedToMsg);
  Serial.println(ssid);
  char ipAddrMsg[30];
  sprintf(ipAddrMsg, "IP address: %s", WiFi.localIP().toString().c_str());
  Serial.println(ipAddrMsg);

}