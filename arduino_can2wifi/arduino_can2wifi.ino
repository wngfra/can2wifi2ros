// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <CAN.h>

#define CS_PIN 3
#define INT_PIN 7

#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

#include "arduino_secrets.h"
// enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your network password (use for WPA, or use as key for WEP)

int status = WL_IDLE_STATUS;
unsigned int localPort = 2390; // local port to listen on
unsigned int remotePort = 10240;
IPAddress remoteIp = IPAddress(192, 168, 0, 100);

WiFiUDP Udp;

const unsigned char IND[16] = {11, 15, 14, 12, 9, 13, 8, 10, 6, 7, 4, 5, 2, 0, 3, 1};

int rxId = 0;
int packetSize;
int rxBuf[8];
char txMsg[33];
unsigned char count = 0;

void setup()
{
  Serial.begin(115200);
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE)
  {
    while (true)
      ;
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED)
  {
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 3 seconds for connection:
    delay(3000);
  }
  // Serial.println("Connected to wifi");
  Udp.begin(localPort);

  CAN.setPins(CS_PIN, INT_PIN);

  CAN.filter(0x400, 0x7f0);

  // Start the CAN bus at 1 Mbps
  while (!CAN.begin(1000E3))
    ;
}

void loop()
{
  // try to parse packet
  packetSize = CAN.parsePacket();
  rxId = CAN.packetId();
  if (rxId == 0x405) {
    Serial.println(rxId, HEX);
    decode(0);
    Serial.println();
  } else if (rxId == 0x407) {
    Serial.println(rxId, HEX);
    decode(4);
    Serial.println();
  } else if (rxId == 0x409) {
    Serial.println(rxId, HEX);
    decode(8);
    Serial.println();
  } else if (rxId == 0x40b) {
    Serial.println(rxId, HEX);
    decode(12);
    Serial.println();
  }

}

// Decode CAN message from bytes
inline void decode(unsigned char ord) {
  for (unsigned char i = 0; i < 4; ++i) {
    // Load CAN message to local buffer
    // rxBuf[i] = CAN.read();
    Serial.print(CAN.read() + 256 * CAN.read());
    Serial.print(" ");
  }
}
