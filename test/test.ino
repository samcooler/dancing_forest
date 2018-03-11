#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN 6

#define N 60

Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);

Adafruit_LIS3DH lis = Adafruit_LIS3DH();

void setup() {
  // Serial.begin(9600);

  // LED Strip:
  #if defined (__AVR_ATtiny85__)
    if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
  #endif
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // accelerometer
  if (! lis.begin(0x18)) {   // change this to 0x19 for alternative i2c address
    Serial.println("Couldnt start");
    while (1);
  }  

  lis.setRange(LIS3DH_RANGE_4_G);   // 2, 4, 8 or 16 G!

}

uint32_t Wheel(byte WheelPos) {
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85) {
      return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if(WheelPos < 170) {
      WheelPos -= 85;
      return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

//int[] before = new int[strip.numPixels()] ;
//uint8_t N = strip.numPixels();
//int N = 60;
int before[N];
int current[N];

boolean first = true;
void loop() {
  sensors_event_t event; 
  lis.getEvent(&event);

  float scale = 20;
  float baseline = 1;
  uint8_t xValue = baseline + scale * abs(event.acceleration.x);
  uint8_t yValue = baseline + scale * abs(event.acceleration.y);
  uint8_t zValue = baseline + scale * abs(event.acceleration.z - 9.8);

  
  current[0] = Wheel(xValue + yValue + zValue);
  current[59] = Wheel(xValue + yValue + zValue); 
  for(uint16_t i=1; i<N/2; i++) {
    current[i] = before[i - 1];
    current[(60-i-1)] = before[60-i];
  }
  
  for(uint16_t i=0; i<strip.numPixels(); i++) {
   
    
    //Serial.print("\t\tX: "); Serial.print(event.acceleration.x); 
    //Serial.print("\t "); Serial.print(xValue);
    //Serial.print("\t "); Serial.print(current[0]);
    //Serial.print("\t "); Serial.print(current[1]);
    //Serial.print("\n");
    //if(i == 0){
    //  strip.setPixelColor(i, strip.Color(255, 0, 0));
    //}else{
    //  strip.setPixelColor(i, strip.Color(0, 255, 0));
    //}
    strip.setPixelColor(i, current[i]);
  }
  for(uint16_t i=0; i<strip.numPixels(); i++) {
      before[i] = current[i];
   }
   strip.show();
   delay(50);
  
}

