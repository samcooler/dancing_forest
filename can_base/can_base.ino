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
  
  if (CAN_MSGAVAIL != CAN.checkReceive())           // check if data coming
  {
    return;
  }
  rcvTime = millis();
  CAN.readMsgBuf(&len, buf);    // read data,  len: data length, buf: data buf

  rxId = CAN.getCanId();

  Serial.print(rcvTime);
  Serial.print("\t\t");
  Serial.print("from: ");
  Serial.print(rxId, HEX);
  Serial.print("\t");

  Serial.print("Rx: Node ");
  Serial.print(rxId - 1);
  Serial.print(" | accel ");
  Serial.print(buf[0]);
  Serial.print(" ");
  Serial.print(buf[1]);
  Serial.print(" ");
  Serial.print(buf[2]);
  
  Serial.println();

//  canMessage[0] = xValue;
//  canMessage[1] = yValue;
//  canMessage[2] = zValue;
//
//  CAN.sendMsgBuf(address, 0, sizeof(canMessage), canMessage);

}
