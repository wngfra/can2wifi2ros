// Copyright (c) 2020 wngfra
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <MCP2515.h>

const unsigned char ID_TABLE[16] = {11, 15, 14, 12, 9, 13, 8, 10, 6, 7, 4, 5, 2, 0, 3, 1};

int rxId = 0;
byte txMsg[32];
unsigned int i, j;
unsigned char id;
unsigned char count = 0;
unsigned char rxBuf[8];

void setup()
{
CAN.filter(0x400, 0x7f0);

  // Start the CAN bus at 1 Mbps
  while (!CAN.begin(1000E3))
    ;
  Serial.begin(1152000);
}

void loop()
{
  if (CAN.parsePacket() == 8) {
    rxId = CAN.packetId();
    if (rxId == 0x405 && count == 0) {
      decode(0);
      ++count;
    } else if (rxId == 0x407 && count == 1) {
      decode(4);
      ++count;
    } else if (rxId == 0x409 && count == 2) {
      decode(8);
      ++count;
    } else if (rxId == 0x40b) {
      decode(12);
      count = 0;

      Serial.write(txMsg, 32);
    }
  }
}

// Decode CAN message from bytes
inline void decode(unsigned char ord) {
  for (i = 0; i < 8; ++i) {
    // Load CAN message to local buffer
    rxBuf[i] = CAN.read();
  }
  for (j = 0; j < 4; ++j) {
    // Re-arrange the data following the taxtile order
    id = ID_TABLE[ord + j];
    txMsg[2 * id] = rxBuf[2 * j + 1];
    txMsg[2 * id + 1] = rxBuf[2 * j];
  }
}