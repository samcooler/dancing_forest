#include <SPI.h>
#include "mcp_can.h"

long unsigned int rxId;
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);
uint8_t incomingMessageLength = 0;
uint8_t incomingMessageBuffer[8];
uint8_t canMessage[5] = {0, 0, 0, 0, 0};

uint8_t nodeData[2][3] = {{0, 0, 0}, {0, 0, 0}};

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
  CAN.readMsgBuf(&incomingMessageLength, incomingMessageBuffer);    // read data,  len: data length, buf: data buf

  rxId = CAN.getCanId();

  Serial.print("Rx: Node ");
  uint8_t nodeId = rxId - 1;
  Serial.print(nodeId);
  Serial.print(" | accel ");
  Serial.print(incomingMessageBuffer[0]);
  Serial.print(" ");
  Serial.print(incomingMessageBuffer[1]);
  Serial.print(" ");
  Serial.print(incomingMessageBuffer[2]);
  Serial.print(" ");
  Serial.print(incomingMessageBuffer[3]);
  Serial.println();

  // incomingMessageBuffer[0] is message type
  nodeData[nodeId][0] = incomingMessageBuffer[1];
  nodeData[nodeId][1] = incomingMessageBuffer[2];
  nodeData[nodeId][2] = incomingMessageBuffer[3];

  // get a message ready to send to the other node
  uint8_t otherNode = (nodeId + 1) % 2;
  canMessage[0] = nodeId; // destination node ID
  canMessage[1] = 2; // feedback data from base
  canMessage[2] = nodeData[nodeId][0];
  canMessage[3] = nodeData[nodeId][1];
  canMessage[4] = nodeData[nodeId][2];


  CAN.sendMsgBuf(0, 0, sizeof(canMessage), canMessage);
  Serial.print("Tx: Base to Node ");
  Serial.print(otherNode);
  Serial.print(" | msg ");
  Serial.print(canMessage[0]);
  Serial.print(" ");
  Serial.print(canMessage[1]);
  Serial.print(" ");
  Serial.print(canMessage[2]);
  Serial.print(" ");
  Serial.print(canMessage[3]);  
  Serial.print(" ");
  Serial.print(canMessage[4]);    
  Serial.println();
}
