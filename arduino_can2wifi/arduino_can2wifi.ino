// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <CAN.h>

#define CS_PIN 3
#define INT_PIN 7

#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>

#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your network password (use for WPA, or use as key for WEP)

int status = WL_IDLE_STATUS;
unsigned int localPort = 2390; // local port to listen on
unsigned int remotePort = 10240;
IPAddress remoteIp = IPAddress(192, 168, 0, 100);

WiFiUDP Udp;

const unsigned char CH_ORD[16] = {11, 15, 14, 12, 9, 13, 8, 10, 6, 7, 4, 5, 2, 0, 3, 1};

int rxId = 0;
unsigned char count = 0;
unsigned char id;
unsigned char rxBuf[8];
unsigned char txMsg[33];

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

  // Set filters
  CAN.filter(0x400, 0x7f0);

  // Start the CAN bus at 1 Mbps
  while (!CAN.begin(1000E3))
    ;
}

// Decode CAN message from bytes to unsigned char arrays
inline void bytes2int(unsigned char id)
{
  // Read CAN buffer to local buffer
  for (int i = 0; i < 8; ++i)
  {
    rxBuf[i] = CAN.read();
  }

  // Rearrange by the sequential taxel order
  for (int j = 0; j < 4; ++j)
  {
    txMsg[CH_ORD[id + j] * 2] = rxBuf[2 * j + 1];
    txMsg[CH_ORD[id + j] * 2 + 1] = rxBuf[2 * j];
  }
}

void publishMsg(unsigned char *msg)
{
  Udp.beginPacket(remoteIp, remotePort);
  Udp.write((char *)msg);
  Udp.endPacket();
}

void loop()
{
  while (count < 4)
  {
    // try to parse packet
    if (CAN.parsePacket() == 8)
    {
      rxId = CAN.packetId();
      if (rxId == 0x405 && count == 0)
      {
        id = 0;
        ++count;
        bytes2int(id);
      }
      else if (rxId == 0x407 && count == 1)
      {
        id = 4;
        ++count;
        bytes2int(id);
      }
      else if (rxId == 0x409 && count == 2)
      {
        id = 8;
        ++count;
        bytes2int(id);
      }
      else if (rxId == 0x40B && count == 3)
      {
        id = 12;
        count = 0;
        bytes2int(id);

        txMsg[32] = '\0';
        publishMsg(txMsg);
      }
    }
  }
}
