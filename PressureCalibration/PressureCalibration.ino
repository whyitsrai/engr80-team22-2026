/********
E80 2026 Team 22 - Pressure Sensor Calibration Sketch

Purpose:
  Output clean, timestamped pressure sensor readings (raw ADC + volts) so you
  can build a calibration curve of voltage vs water column height.

How to use:
  1. Seal the pressure sensor into the bottom of a graduated cylinder or tube.
  2. Upload this sketch. Open Serial Monitor at 115200 baud.
  3. Also open Serial Plotter if you want a live view of the voltage.
  4. Starting with sensor at atmospheric (sensor just submerged, ~0 cm column),
     press USER_BUTTON to mark a sample point. The sketch will:
        - wait 2 seconds for the reading to settle
        - collect 100 samples over ~1 second
        - print a single averaged CSV row
  5. Add water to a known height (10, 20, 30, ... cm), wait for stabilization,
     press the button again. Repeat up to your max depth (~1 m).
  6. Copy the CSV rows into MATLAB and fit: depth_m = slope * voltage + intercept
  7. Paste slope and intercept into ZStateEstimator.h.

Output format (CSV, ready to paste into MATLAB):
  sample_num, height_cm (you fill in), raw_adc_mean, raw_adc_std, voltage_mean, voltage_std

Notes:
  - Since this is absolute pressure, atmospheric baseline varies day-to-day.
    Re-run the full calibration each deployment day if precision matters.
  - The sketch does NOT know the water column height - YOU record it against
    each printed sample_num in your lab notebook or a spreadsheet column.
********/

#include <Arduino.h>
#include <Pinouts.h>

#define N_AVG 500           // samples per button press
#define SAMPLE_DELAY_MS 10  // -> ~1 second per measurement burst
#define SETTLE_MS 2000      // wait time after button press before sampling
#define ADC_MAX 1023.0f     // 10-bit ADC
#define VREF 3.3f           // Teensy 4.0 analog reference

int sampleNum = 0;

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(PRESSURE_PIN, INPUT);
  pinMode(USER_BUTTON, INPUT_PULLUP);

  // Ensure 10-bit ADC (default, but explicit is safer)
  analogReadResolution(10);
  analogReadAveraging(32);
  analogReadAveraging(64);

  Serial.println("=== Pressure Sensor Calibration ===");
  Serial.println("Press USER_BUTTON at each known water column height.");
  Serial.println("Record the height in your notebook against sample_num.");
  Serial.println();
  Serial.println("Live streaming raw ADC and voltage below.");
  Serial.println("Sample readings print as CSV rows when you press the button.");
  Serial.println();
  Serial.println("# sample_num, raw_adc_mean, raw_adc_std, voltage_mean, voltage_std");
}

void loop() {
  // ---------- Live stream (slow, for Serial Plotter / visual check) ----------
  static unsigned long lastStream = 0;
  if (millis() - lastStream > 200) {
    lastStream = millis();
    int raw = analogRead(PRESSURE_PIN);
    float voltage = raw * (VREF / ADC_MAX);
    // Tab-separated so Serial Plotter can parse it
    Serial.print("raw=");
    Serial.print(raw);
    Serial.print("\tvolts=");
    Serial.println(voltage, 4);
  }

  // ---------- Button-triggered sample burst ----------
  if (digitalRead(USER_BUTTON) == LOW) {
    // debounce
    delay(50);
    if (digitalRead(USER_BUTTON) != LOW) return;

    Serial.println();
    Serial.print("# Button pressed. Settling for ");
    Serial.print(SETTLE_MS);
    Serial.println(" ms...");
    delay(SETTLE_MS);

    Serial.print("# Sampling ");
    Serial.print(N_AVG);
    Serial.println(" readings...");

    // Collect
    double sumRaw = 0, sumSqRaw = 0;
    for (int i = 0; i < N_AVG; i++) {
      int raw = analogRead(PRESSURE_PIN);
      sumRaw += raw;
      sumSqRaw += (double)raw * raw;
      delay(SAMPLE_DELAY_MS);
    }

    // Stats
    double meanRaw = sumRaw / N_AVG;
    double varRaw = (sumSqRaw / N_AVG) - meanRaw * meanRaw;
    double stdRaw = sqrt(varRaw > 0 ? varRaw : 0);

    double meanV = meanRaw * (VREF / ADC_MAX);
    double stdV = stdRaw * (VREF / ADC_MAX);

    // Print one CSV row
    Serial.print(sampleNum);
    Serial.print(", ");
    Serial.print(meanRaw, 2);
    Serial.print(", ");
    Serial.print(stdRaw, 3);
    Serial.print(", ");
    Serial.print(meanV, 5);
    Serial.print(", ");
    Serial.println(stdV, 5);

    // Warning on excessive noise
    if (stdRaw > 3.0) {
      Serial.print("# WARNING: high noise (std=");
      Serial.print(stdRaw, 2);
      Serial.println(" counts). Check for bubbles or loose wiring.");
    }

    sampleNum++;

    // Wait for release before resuming stream
    while (digitalRead(USER_BUTTON) == LOW) delay(10);
    Serial.println("# Ready for next sample. Change water height and press again.");
    Serial.println();
  }
}
