#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN 17

#define N 60
#define K 100

Adafruit_NeoPixel strip = Adafruit_NeoPixel(N, PIN, NEO_GRB + NEO_KHZ800);

Adafruit_LIS3DH lis = Adafruit_LIS3DH();

void setup() {
  //Serial.begin(9600);

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

//int K = 100;
float displacement_hx[K];
float color_hx[K];
void loop() {
  sensors_event_t event; 
  lis.getEvent(&event);

  float max_theta = PI/4;
  float displacement = acos(min(event.acceleration.z, 9.8)/9.8)/max_theta;
  if(displacement > 1){
    displacement = 1;
  }
  float angle = atan2(event.acceleration.y, event.acceleration.x);
  // Lets assume for now that the angle is between -pi and pi
  float color_from_angle = (angle + PI)/(2*PI)*255;

  float my_color = 0.0;
  for(uint16_t i=0; i<(K-1); i++) {
    color_hx[i] = color_hx[i+1];
    my_color += color_hx[i];
  }
  color_hx[(K-1)] =  color_from_angle; 
  my_color += color_from_angle;
  my_color /= K;

  float my_disp = 0.0; 
  for(uint16_t i=0; i<(K-1); i++) {
    displacement_hx[i] = displacement_hx[i+1];
    my_disp += displacement_hx[i];
  }
  displacement_hx[(K-1)] = displacement;
  my_disp += displacement;
  my_disp /= K;

  Serial.print("theta: "); Serial.print(angle); 
  Serial.print("\tColor val: "); Serial.print(my_color); 
  Serial.print("\tDisplacement: "); Serial.print(displacement);
  Serial.print("\tAvg Disp: "); Serial.print(my_disp);
  Serial.print("\n") ;
  float max_leds = N * my_disp;
  for(uint16_t i=0; i<max_leds; i++) {
    strip.setPixelColor(i, Wheel(my_color));
  }
  if(max_leds < 60){
    for(uint16_t i = ceil(max_leds); i < strip.numPixels(); i ++ ){
      strip.setPixelColor(i, 0); 
    }
  }
   strip.show();
  
}

