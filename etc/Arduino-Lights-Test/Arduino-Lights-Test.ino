#include <FastLED.h>
#define LED_PIN 3
#define NUM_LEDS 144

CRGB leds[NUM_LEDS];

void setup() {
  // put your setup code here, to run once:
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
}

void loop() {
  // put your main code here, to run repeatedly:
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB(255,255,0);
    }
    FastLED.show();
    delay(40);
}
