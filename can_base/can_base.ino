#include <SPI.h>
#include "mcp_can.h"

long unsigned int rxId;
unsigned long rcvTime;
unsigned char len = 0;
unsigned char buf[8];

const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

void setup() {
  delay(1000);
  Serial.begin(115200);

  Serial.println("Base start");

  // CAN init
  Serial.println("CAN init start");
  if (CAN.begin(CAN_250KBPS) == CAN_OK)
  {
    Serial.println("CAN init ok");
  }
  else Serial.println("CAN init fail!");
}

void loop() {
  
  if (CAN_MSGAVAIL == CAN.checkReceive())           // check if data coming
  {
    rcvTime = millis();
    CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

    rxId = CAN.getCanId();

    Serial.print(rcvTime);
    Serial.print("\t\t");
    Serial.print("0x");
    Serial.print(rxId, HEX);
    Serial.print("\t");

    for (int i = 0; i < len; i++) // print the data
    {
      if (buf[i] > 15) {
        Serial.print("0x");
        Serial.print(buf[i], HEX);
      }
      else {
        Serial.print("0x0");
        Serial.print(buf[i], HEX);
      }

      //Serial.print("0x");
      //Serial.print(buf[i], HEX);

      Serial.print("\t");
    }
    Serial.println();
  }

}
