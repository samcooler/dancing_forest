#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN 17

#define N 60

Adafruit_NeoPixel strip = Adafruit_NeoPixel(N, PIN, NEO_GRB + NEO_KHZ800);

Adafruit_LIS3DH lis = Adafruit_LIS3DH();

void setup() {
  // Serial.begin(9600);

  // LED Strip:
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  strip.begin();
  strip.show();

  // accelerometer
  if (! lis.begin(0x18)) {
    Serial.println("Couldnt start");
    while (1);
  }  

  lis.setRange(LIS3DH_RANGE_4_G);

}

void loop(){
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
}

