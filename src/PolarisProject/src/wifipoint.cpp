#include <WiFi.h>
#include <WiFiUdp.h>
 
const char* ssid = "Polaris";
const char* password = "longhorns";
 
WiFiUDP udp;
const int localUdpPort = 1234;  // Port for receiving
 
 
// define input pins of the L298n Motor Driver pins.
#define in1 5
#define in2 6
#define in3 10
#define in4 11
#define LED 13
 
void wifisetup() {
  Serial.begin(115200);
 
  // Set up the ESP32 as an access point
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point IP: ");
  Serial.println(IP);
 
  // Start UDP server
  udp.begin(localUdpPort);
  Serial.println("UDP server started, waiting for incoming messages...");
}
 
void wifisample() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Allocate buffer to store received data
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = '\0';  // Null-terminate the string
    }
 
    Serial.print("Received message: ");
    Serial.println(incomingPacket);
 
    
  }
}
 
