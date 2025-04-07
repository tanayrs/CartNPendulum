#include <WiFi.h>
#include <WiFiUdp.h>

#define RXp2 16
#define TXp2 17

const char* ssid = "Guest";
const char* password = "Holiday#2023@";
WiFiUDP udp;
const int udpPort = 4210;

// Add a delay between broadcasts to prevent overwhelming the system
const unsigned long BROADCAST_INTERVAL = 100; // milliseconds
unsigned long lastBroadcastTime = 0;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXp2, TXp2);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  
  Serial.println("\nConnected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  // Start UDP
  udp.begin(udpPort);
}

void loop() {
  // Check WiFi connection and reconnect if needed
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Reconnecting...");
    WiFi.reconnect();
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nReconnected to WiFi");
  }
  
  // Process serial data with rate limiting
  unsigned long currentTime = millis();
  if (Serial2.available() && (currentTime - lastBroadcastTime >= BROADCAST_INTERVAL)) {
    String message = Serial2.readStringUntil('\n'); // Read until newline
    message.trim(); // Remove any whitespace
    
    if (message.length() > 0) {
      Serial.println("Broadcasting: " + message);
      
      // Broadcast to all devices on the local network
      udp.beginPacket("255.255.255.255", udpPort);
      udp.print(message);
      udp.endPacket();
      
      lastBroadcastTime = currentTime;
    }
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}