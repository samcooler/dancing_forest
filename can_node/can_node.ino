#include "mcp_can.h"
#include <SPI.h>
//#include <Adafruit_NeoPixel.h>
#include <Adafruit_DotStar.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>


// little script to enable reading of RAM remaining amount
#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__

int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}

// program setup
char state = 'r';

// CAN setup
#define extendedMode 0
uint8_t canMessage[5] = {0, 0, 0, 0, 0};
uint8_t incomingMessageLength = 0;
uint8_t incomingMessageBuffer[8];
uint8_t dataFromBase[8] = {0, 0, 0, 0, 0, 0, 0, 0};
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

// LED setup
#ifdef __AVR__
#include <avr/power.h>
#endif

#define NUMPIXELS      60
#define LED_DATAPIN    8
#define LED_CLOCKPIN   7
//Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN_LED, NEO_GRB + NEO_KHZ800);
Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, LED_DATAPIN, LED_CLOCKPIN, DOTSTAR_BRG);

// Accel setup
Adafruit_LIS3DH lis = Adafruit_LIS3DH();


// address setup
const int PIN_address = 4;
uint8_t address = 0;


// display setup
uint32_t before[NUMPIXELS];
uint32_t current[NUMPIXELS];

void setup()
{
  delay(1000);
  Serial.begin(115200);

  Serial.println(F("Node start"));


  // address init
  pinMode(PIN_address, INPUT);
  address = digitalRead(PIN_address) + 1; // base has the first address 0x00

  Serial.print("Node Id: ");
  Serial.println(address);

  // CAN init
  Serial.println(F("CAN init start"));
  if (CAN.begin(CAN_250KBPS) == CAN_OK)
  {
    Serial.println(F("CAN init ok"));
  }
  else Serial.println(F("CAN init fail!"));

  // LED init
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  strip.begin();

  // Accel init
  if (! lis.begin(0x18)) {
    Serial.println(F("Accel init fail"));
  }
  Serial.println(F("Accel init ok"));
  lis.setRange(LIS3DH_RANGE_4_G);


  Serial.println(F("Done with init"));
}




void loop()
{
  // Receive data from base
  if (CAN_MSGAVAIL == CAN.checkReceive())
  {
    Serial.print(F("got message from CAN ID "));
    CAN.readMsgBuf(&incomingMessageLength, incomingMessageBuffer);
    uint32_t fromAddress = CAN.getCanId();
    uint8_t toAddress = incomingMessageBuffer[0];
    // if it's not a message for us, ignore it
    Serial.println(fromAddress);
    Serial.print(" for node ");
    Serial.println(toAddress);
    if (toAddress == address)
    {

      // for now, just stay in "run" mode
      Serial.print(F("Rx "));
      Serial.print(incomingMessageLength);
      Serial.print(": ");
      for (uint8_t i = 0; i < incomingMessageLength; i++) {
        Serial.print(incomingMessageBuffer[i]);
        Serial.print(" ");
      }
      Serial.println();

      switch (incomingMessageBuffer[1])
      {
        case 2:
          Serial.println(F("Got message from base"));
          state = 'r';
          for (uint8_t i = 2; i < incomingMessageLength; i++) {
            dataFromBase[i - 2] = incomingMessageBuffer[i];
          }
      }
    }

  }


  switch (state)
  {
    case 'r':
      //      Serial.println("State: run");
      // read accelerometer
      sensors_event_t event;
      lis.getEvent(&event);


      // setup data to send to base
      float scale = 20;
      float baseline = 128;
      canMessage[1] = baseline + scale * event.acceleration.x;
      canMessage[2] = baseline + scale * event.acceleration.y;
      canMessage[3] = baseline + scale * (event.acceleration.z - 9.8);

      /*
        message format:
        messageType, [data]

        messageType: 0 "announce presence, wait for setup"
        messageType: 1 "sensor data"
        data: {x, y, z} from accelerometer
        messageType: 2 "feedback data"
        data: {h, l, s} or arbitrary numerical values
      */

      canMessage[0] = 1; // sensor data
      CAN.sendMsgBuf(address + 1, extendedMode, sizeof(canMessage), canMessage);



      // display code here:
      scale = .4;
      baseline = 1;
      uint8_t xValue = baseline + scale * abs(dataFromBase[0] - 128);
      uint8_t yValue = baseline + scale * abs(dataFromBase[1] - 128);
      uint8_t zValue = baseline + scale * abs(dataFromBase[2] - 128);

//      for (uint16_t i = 0; i < strip.numPixels(); i++) {
//        strip.setPixelColor(i, strip.Color(xValue, yValue, zValue));
//      }
//      strip.show();



      current[0] = strip.Color(xValue, yValue, zValue);
      current[59] = current[0];
      for (uint8_t i = 1; i < NUMPIXELS / 2; i++) {
        current[i] = before[i - 1];
        current[(NUMPIXELS - i - 1)] = before[NUMPIXELS - i];
      }

      for (uint8_t i = 0; i < NUMPIXELS; i++) {
        //Serial.print("\t\tX: "); Serial.print(event.acceleration.x);
        //Serial.print("\t "); Serial.print(xValue);
        //Serial.print("\t "); Serial.print(current[0]);
        //Serial.print("\t "); Serial.print(current[1]);
        //Serial.print("\n");
        strip.setPixelColor(i, current[i]);
      }
      for (uint8_t i = 0; i < NUMPIXELS; i++) {
        before[i] = current[i];
      }
      strip.show();

      // END CASE 'r'
  }

//  Serial.println(freeMemory());
  delay(10);
}
