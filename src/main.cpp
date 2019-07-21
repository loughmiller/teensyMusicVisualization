#include <Arduino.h>
#define ARM_MATH_CM4
#include <arm_math.h>
#include <FastLED.h>
#include <algorithm>    // std::sort

using namespace std; 

// FAST LED
#define DISPLAY_LED_PIN 4            // VERSION 2
#define AUDIO_INPUT_PIN A7           // MEDALLION
const int numLEDs{24};               // MEDALLION


////////////////////////////////////////////////////////////////////////////////////////////////
// NOTE DETECTION
////////////////////////////////////////////////////////////////////////////////////////////////

// NOTE DETECTION CONSTANTS
const uint_fast16_t fftSize{256};               // Size of the FFT.  Realistically can only be at most 256
const uint_fast16_t fftBinSize{8};              // Hz per FFT bin  -  sample rate is fftSize * fftBinSize
const uint_fast16_t sampleCount{fftSize * 2};   // Complex FFT functions require a coefficient for the imaginary part of the
                                                // input.  This makes the sample array 2x the fftSize
const float middleA{440.0};                     // frequency of middle A.  Needed for freqeuncy to note conversion
const uint_fast16_t sampleIntervalMs{1000000 / (fftSize * fftBinSize)};  // how often to get a sample, needed for IntervalTimer

// FREQUENCY TO NOTE CONSTANTS - CALCULATE HERE: https://docs.google.com/spreadsheets/d/1CPcxGFB7Lm6xJ8CePfCF0qXQEZuhQ-nI1TC4PAiAd80/edit?usp=sharing
const uint_fast16_t noteCount{41};              // how many notes are we trying to detect
const uint_fast16_t notesBelowMiddleA{30};      

// NOTE DETECTION GLOBALS
float samples[sampleCount*2];
uint_fast16_t sampleCounter = 0;
float sampleBuffer[sampleCount];
float magnitudes[fftSize];
float noteMagnatudes[noteCount];
arm_cfft_radix4_instance_f32 fft_inst;
IntervalTimer samplingTimer;

// NOTE DETECTION FUNCTIONS
void noteDetectionSetup();        // run this once during setup
void noteDetectionLoop();         // run this once per loop
void samplingCallback();

////////////////////////////////////////////////////////////////////////////////////////////////
// \ NOTE DETECTION
////////////////////////////////////////////////////////////////////////////////////////////////

CRGB leds[numLEDs];
const int saturation{244};
int hue = 0;

void setup() {
  delay(3000);

  Serial.begin(9600);
  Serial.println("setup started");

  noteDetectionSetup();

  // DISPLAY STUFF
  FastLED.addLeds<NEOPIXEL, DISPLAY_LED_PIN>(leds, numLEDs).setCorrection( TypicalLEDStrip );;
  delay(1000);

  Serial.println("setup complete");
}

int time = -1;
int iteration = 0;
float threshold = 500.0;
float peak = 2000.0;

void loop() {
  bool newSec = false;
  hue+=3;
  iteration++;

  noteDetectionLoop();

  float sorted[noteCount];
  memcpy(sorted, noteMagnatudes, sizeof(noteMagnatudes[0]) * noteCount);
  sort(sorted, sorted+sizeof(sorted)/sizeof(sorted[0]));

  float twoThirds = sorted[(uint_fast16_t) (0.75 * noteCount)];
  float maxNote = sorted[noteCount - 1];
  threshold = (threshold * (0.99)) + (twoThirds/100.0);
  peak = (peak * (0.99)) + (maxNote/100.0);

  if (newSec) {
    Serial.println(twoThirds);
    Serial.print("threshold: ");
    Serial.println(threshold);
    Serial.println(maxNote);
    Serial.print("peak: ");
    Serial.println(peak);
  }

  for (uint_fast16_t i=0; i<noteCount; i++) {
    if (noteMagnatudes[i] >= threshold) {
      int value = 0;
      if (noteMagnatudes[i] > peak) {
        value = 255;
      } else {
        value = ((noteMagnatudes[i] - threshold) / (peak - threshold)) * 255;
      }

      leds[i % 24] = CHSV(hue, saturation, value);
    } else {
      leds[i % 24] = CRGB::Black;
    }
  }

   FastLED.show(); 
}

////////////////////////////////////////////////////////////////////////////////////////////////
// NOTE DETECTION
////////////////////////////////////////////////////////////////////////////////////////////////

void noteDetectionSetup() {
  pinMode(AUDIO_INPUT_PIN, INPUT);
  analogReadResolution(10);
  analogReadAveraging(16);
  arm_cfft_radix4_init_f32(&fft_inst, fftSize, 0, 1);
  samplingTimer.begin(samplingCallback, sampleIntervalMs);
}

void noteDetectionLoop() {
  // copy the last N samples into a buffer
  memcpy(sampleBuffer, samples + (sampleCounter + 1), sizeof(float) * sampleCount);

  // FFT magic
  arm_cfft_radix4_f32(&fft_inst, sampleBuffer);
  arm_cmplx_mag_f32(sampleBuffer, magnitudes, fftSize);

  for (uint_fast16_t i=0; i<noteCount; i++) {
    noteMagnatudes[i] = 0;
  }

  for (uint_fast16_t i=1; i<fftSize/2; i++) {  // ignore top half of the FFT results
    float frequency = i * (fftBinSize);
    int note = roundf(12 * (log(frequency / middleA) / log(2))) + notesBelowMiddleA;

    if (note < 0) {
      continue;
    }

    note = note % noteCount;
    noteMagnatudes[note] = max(noteMagnatudes[note], magnitudes[i]);
  }  
}

void samplingCallback() {
  // Read from the ADC and store the sample data
  float sampleData = (float)analogRead(AUDIO_INPUT_PIN);

  // storing the data twice in the ring buffer array allows us to do a single memcopy
  // to get an ordered buffer of the last N samples
  uint_fast16_t sampleIndex = (sampleCounter) * 2;
  uint_fast16_t sampleIndex2 = sampleIndex + sampleCount;
  samples[sampleIndex] = sampleData;
  samples[sampleIndex2] = sampleData;

  // Complex FFT functions require a coefficient for the imaginary part of the
  // input.  Since we only have real data, set this coefficient to zero.
  samples[sampleIndex+1] = 0.0;
  samples[sampleIndex2+1] = 0.0;

  sampleCounter++;
  sampleCounter = sampleCounter % fftSize;
}

////////////////////////////////////////////////////////////////////////////////////////////////
// \ NOTE DETECTION
////////////////////////////////////////////////////////////////////////////////////////////////