#include <CAN.h>

#define CS_PIN 3
#define INT_PIN 7

const unsigned char CH_ORD[16] = {11, 15, 14, 12, 9, 13, 8, 10, 6, 7, 4, 5, 2, 0, 3, 1};

int rxId = 0;
int rxVal[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
unsigned char count = 0;
unsigned char id;
unsigned char packetSize;
unsigned char rxBuf[8];

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println("CAN Receiver");
  CAN.setPins(CS_PIN, INT_PIN);

  // Set filters
  CAN.filter(0x400, 0x7f0);

  // Start the CAN bus at 1 Mbps
  if (!CAN.begin(1000E3))
  {
    Serial.println("Starting CAN failed!");
    while (1)
      ;
  }
}

void bytes2int(unsigned char id)
{
  for (int i = 0; i < 8; ++i)
  {
    rxBuf[i] = CAN.read();
  }

  for (int j = 0; j < 4; ++j)
  {
    rxVal[CH_ORD[id + j]] = (int)(rxBuf[2 * j + 1] << 8) + (int)(rxBuf[2 * j]);
  }

  if (id == 12)
  {
    Serial.print("CAN read: ");
    for (int k = 0; k < 16; ++k)
    {
      Serial.print(rxVal[k]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void loop()
{
  while (count < 4)
  {
    // try to parse packet
    packetSize = CAN.parsePacket();
    rxId = CAN.packetId();

    if (packetSize == 8)
    {
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
      }
    }
  }
}