#include <FastLED.h>

#define LED_PIN     37
#define NUM_LEDS    1       // change if you have more LEDs
#define DELAY_MS    800     // time each colour stays on

CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(100);   // 0-255
}

void loop() {
  // Red
  leds[0] = CRGB(255, 0, 0);
  FastLED.show();
  delay(DELAY_MS);

  // Green
  leds[0] = CRGB(0, 255, 0);
  FastLED.show();
  delay(DELAY_MS);

  // Purple
  leds[0] = CRGB(148, 0, 211);
  FastLED.show();
  delay(DELAY_MS);

  // Orange
  leds[0] = CRGB(255, 100, 0);
  FastLED.show();
  delay(DELAY_MS);
}