#include "mcp_can.h"
#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

// CAN setup
uint8_t canMessage[5] = {0, 0, 0, 0, 0};
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

// LED setup
#ifdef __AVR__
#include <avr/power.h>
#endif
#define PIN_LED       6
#define NUMPIXELS      1
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN_LED, NEO_GRB + NEO_KHZ800);

// Accel setup
Adafruit_LIS3DH lis = Adafruit_LIS3DH();


// address setup
const int PIN_address = 4;
uint8_t address = 0;


void setup()
{
  delay(1000);
  Serial.begin(115200);

  Serial.println("Node start");

  // address init
  pinMode(PIN_address, INPUT);
  address = digitalRead(PIN_address);
  canMessage[1] = address;
  
  // CAN init
  Serial.println("CAN init start");
  if (CAN.begin(CAN_250KBPS) == CAN_OK)
  {
    Serial.println("CAN init ok");
  }
  else Serial.println("CAN init fail!");

  // LED init
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  strip.begin();

  // Accel init
  if (! lis.begin(0x18)) {
    Serial.println("Accel init fail");
  }
  Serial.println("Accel init ok");
  lis.setRange(LIS3DH_RANGE_4_G);
}

void loop()
{
  // read accelerometer

  sensors_event_t event;
  lis.getEvent(&event);

  float scale = 20;
  float baseline = 1;
  uint8_t xValue = baseline + scale * abs(event.acceleration.x);
  uint8_t yValue = baseline + scale * abs(event.acceleration.y);
  uint8_t zValue = baseline + scale * abs(event.acceleration.z - 9.8);

  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(xValue, yValue, zValue));
  }
  strip.show();
  
  canMessage[2] = xValue;
  canMessage[3] = yValue;
  canMessage[4] = zValue;

  CAN.sendMsgBuf(0x00, 0, 8, canMessage);
  delay(100);


}
