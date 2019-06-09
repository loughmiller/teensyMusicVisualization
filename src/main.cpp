#include <Arduino.h>
#define ARM_MATH_CM4
#include <arm_math.h>
#include <FastLED.h>

// FAST LED
#define NUM_LEDS 24                  // MEDALLION
#define DISPLAY_LED_PIN 4            // VERSION 2
#define AUDIO_INPUT_PIN A7           // MEDALLION

#define SAMPLE_RATE_HZ 2560          // Sample rate of the audio in hertz.
#define FFT_SIZE 256                 // Size of the FFT.  Realistically can only be at most 256
                                     // 10Hz per bin

#define MIDDLE_A 440.0

#define SAMPLE_SIZE FFT_SIZE*2       // Complex FFT functions require a coefficient for the imaginary part of the
                                     // input.  This makes the sample array 2x the FFT_SIZE
#define ANALOG_READ_RESOLUTION 10    // Bits of resolution for the ADC.
#define ANALOG_READ_AVERAGING 16     // Number of samples to average with each ADC reading.


CRGB leds[NUM_LEDS];
#define SATURATION 244

uint16_t sampleCounter = 0;
float samples[SAMPLE_SIZE*2];
float magnitudes[FFT_SIZE];
float foo[SAMPLE_SIZE];

IntervalTimer samplingTimer;

void samplingCallback() {
  // Read from the ADC and store the sample data
  uint16_t sampleIndex = (sampleCounter) * 2;
  uint16_t sampleIndex2 = sampleIndex + (FFT_SIZE * 2);
  float sampleData = (float)analogRead(AUDIO_INPUT_PIN);
  samples[sampleIndex] = sampleData;
  samples[sampleIndex2] = sampleData;
  // Complex FFT functions require a coefficient for the imaginary part of the
  // input.  Since we only have real data, set this coefficient to zero.
  samples[sampleIndex+1] = 0.0;
  samples[sampleIndex2+1] = 0.0;

  // increment sample counter for next sample
  sampleCounter++;
  sampleCounter = sampleCounter % FFT_SIZE;

  // Serial.print("sample: ");
  // Serial.print(sampleIndex);
  // Serial.print(" ");
  // Serial.print(sampleIndex2);
  // Serial.print(" ");
  // Serial.println(sampleData);
}

void setup() {
  delay(500);

  Serial.begin(9600);
  Serial.println("setup started");

  randomSeed(analogRead(15));

  Serial.println(AUDIO_INPUT_PIN);

  pinMode(AUDIO_INPUT_PIN, INPUT);
  analogReadResolution(ANALOG_READ_RESOLUTION);
  analogReadAveraging(ANALOG_READ_AVERAGING);

  samplingTimer.begin(samplingCallback, 1000000/SAMPLE_RATE_HZ);

  delay(1000);

  Serial.println("setup complete");
}

void loop() {
  memcpy(foo, samples + (sampleCounter + 1), sizeof(float) * SAMPLE_SIZE);
  arm_cfft_radix4_instance_f32 fft_inst;
  arm_cfft_radix4_init_f32(&fft_inst, FFT_SIZE, 0, 1);
  arm_cfft_radix4_f32(&fft_inst, foo);
  // Calculate magnitude of complex numbers output by the FFT.
  arm_cmplx_mag_f32(foo, magnitudes, FFT_SIZE);

  int j = 1;
  for(int i=1; i<FFT_SIZE/2; i++) {
    if (magnitudes[i] > magnitudes[j]) {
      j = i;
    }
  }

  float frequency = j * (SAMPLE_RATE_HZ / FFT_SIZE);

  int note = roundf(12 * (log(frequency / MIDDLE_A) / log(2)));
  
  Serial.print(j);
  Serial.print(" - ");
  Serial.print(note);
  Serial.print(" - ");
  Serial.print(frequency);
  Serial.print(" - ");
  Serial.println(magnitudes[j]);

  delay(200);
}
