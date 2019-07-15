#include <Arduino.h>
#define ARM_MATH_CM4
#include <arm_math.h>
#include <FastLED.h>
#include <algorithm>    // std::sort
#include <Visualization.h>

using namespace std; 

// FAST LED
#define DISPLAY_LED_PIN 4            // VERSION 2
#define AUDIO_INPUT_PIN A7           // MEDALLION
const int numLEDs{24};               // MEDALLION


// FFT CONSTANTS
const uint_fast16_t fftSize{256};               // Size of the FFT.  Realistically can only be at most 256
const uint_fast16_t fftBinSize{8};              // Hz per FFT bin  -  sample rate is fftSize * fftBinSize
const uint_fast16_t sampleCount{fftSize * 2};   // Complex FFT functions require a coefficient for the imaginary part of the
                                                // input.  This makes the sample array 2x the fftSize
const float middleA{440.0};                     // frequency of middle A.  Needed for freqeuncy to note conversion
const uint_fast16_t sampleIntervalMs{1000000 / (fftSize * fftBinSize)};  // how often to get a sample, needed for IntervalTimer

// FFT GLOBALS
float samples[sampleCount*2];
uint_fast16_t sampleCounter = 0;

// FFT FUNCTIONS
void samplingCallback();


float sampleBuffer[sampleCount];
float magnitudes[fftSize];

float noteMagnatudes[numLEDs];
CRGB leds[numLEDs];
const int saturation{244};


int hue = 0;

Visualization * all;

IntervalTimer samplingTimer;


void setup() {
  delay(3000);

  Serial.begin(9600);
  Serial.println("setup started");

  randomSeed(analogRead(15));

  // FFT SETUP
  pinMode(AUDIO_INPUT_PIN, INPUT);
  analogReadResolution(10);
  analogReadAveraging(16);
  samplingTimer.begin(samplingCallback, sampleIntervalMs);

  // DISPLAY STUFF
  FastLED.addLeds<NEOPIXEL, DISPLAY_LED_PIN>(leds, numLEDs).setCorrection( TypicalLEDStrip );;
  all = new Visualization(numLEDs, 1, hue, saturation, leds);
  FastLED.setBrightness(32);
  FastLED.setDither(0);
  all->setSaturation(saturation);
  all->setAllCRGB(0x101010);
  FastLED.show();
  delay(1000);

  Serial.println("setup complete");
}

int time = -1;
int iteration = 0;
float threshold = 1000.0;
float peak = 2000.0;

void loop() {
  // Serial.println("Start Loop");
  all->setAllCRGB(0x000000);  // this only clears the array, not the LEDs, it's fine at the top

  hue+=3;
  all->setHue(hue);

  bool newSec = false;
  int newTime = millis() / 1000;
  // if (newTime > time) {
  //   newSec = true;
  //   time = newTime;
  //   Serial.print("foo: ");
  //   Serial.print(time);
  //   Serial.print(" - ");
  //   Serial.println(iteration);
  // }
  iteration++;

  memcpy(sampleBuffer, samples + (sampleCounter + 1), sizeof(float) * sampleCount);
  arm_cfft_radix4_instance_f32 fft_inst;
  arm_cfft_radix4_init_f32(&fft_inst, fftSize, 0, 1);
  arm_cfft_radix4_f32(&fft_inst, sampleBuffer);
  // Calculate magnitude of complex numbers output by the FFT.
  arm_cmplx_mag_f32(sampleBuffer, magnitudes, fftSize);

  for (uint_fast16_t i=0; i<numLEDs; i++) {
    noteMagnatudes[i] = 0;
  }

  for (uint_fast16_t i=1; i<fftSize/2; i++) {  // ignore top half of the FFT results
    float frequency = i * (fftBinSize);
    int note = roundf(12 * (log(frequency / middleA) / log(2))) + 32;
    note = note % numLEDs;
    // Serial.print(note);
    // Serial.print(" - ");
    // Serial.print(frequency);
    // Serial.print(" - ");
    // Serial.println(magnitudes[i]);

    noteMagnatudes[note] = max(noteMagnatudes[note], magnitudes[i]);
  }

  // for (int i=0; i<numLEDs; i++) {
  //   Serial.print(i);
  //   Serial.print(" - ");
  //   Serial.println(noteMagnatudes[i]);
  // }
  // Serial.println();

  float sorted[numLEDs];
  memcpy(sorted, noteMagnatudes, sizeof(noteMagnatudes[0]) * numLEDs);
  sort(sorted, sorted+sizeof(sorted)/sizeof(sorted[0]));

  float twoThirds = sorted[16];
  float maxNote = sorted[23];
  threshold = (threshold * (99.0/100.0)) + (twoThirds/100.0);
  peak = (peak * (99.0/100.0)) + (maxNote/100.0);
  // Serial.print("2/3rd: ");

  if (newSec) {
    Serial.println(twoThirds);
    Serial.print("threshold: ");
    Serial.println(threshold);
    Serial.println(maxNote);
    Serial.print("peak: ");
    Serial.println(peak);

  //   for (int i=0; i<numLEDs; i++) {
  //     Serial.print(i);
  //     Serial.print(" - ");
  //     Serial.println(sorted[i]);
  //   }
  }

  for (uint_fast16_t i=0; i<numLEDs; i++) {
    if (noteMagnatudes[i] >= threshold) {
      int value = 0;
      if (noteMagnatudes[i] > peak) {
        value = 255;
      } else {
        value = ((noteMagnatudes[i] - threshold) / (peak - threshold)) * 256;
      }
      // Serial.println(value);
      all->setValue(value);
      all->setLEDColor(i);
    }
  }

  // Serial.print(j);
  // Serial.print(" - ");
  // Serial.print(note);
  // Serial.print(" - ");
  // Serial.print(frequency);
  // Serial.print(" - ");
  // Serial.println(magnitudes[j]);

  // delay(1000);
   FastLED.show(); 
  //  Serial.print(threshold);
  //  Serial.print(" - ");
  //  Serial.println(noteMagnatudes[10]);
  // Serial.println("End Loop\n");
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