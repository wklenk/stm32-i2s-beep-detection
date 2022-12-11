/*
    Detect beeps of washing machine or dryer and notify via push notication
    on a smartphone. Uses digital I2S protocol with microphone.

    Copyright (C) 2022 Wolfgang Klenk (wolfgang.klenk@gmail.com)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

// Include I2S driver
#include <driver/i2s.h>
#include <WiFi.h>
#include <arduinoFFT.h>

// #include "Pushover.h"

// Connections to INMP441 I2S microphone
#define I2S_WS 25
#define I2S_SD 22
#define I2S_SCK 26

// Use I2S Processor 0
#define I2S_PORT I2S_NUM_0

// Define I2S input buffer length
#define bufferLen 64  // 8 x 8 samples
int16_t sBuffer[bufferLen];

// Replace with your network credentials
const char* ssid = "your ssid";
const char* password = "your WLAN password";
String hostname = "ESP32 Beep Detection";

uint16_t current_sample = 0;
const uint16_t SAMPLES = 512;  //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 44100;

double stable_frequency;
boolean is_stable = false;
unsigned long frequency_stable_start;

// Arduino FFT library
arduinoFFT FFT = arduinoFFT(); 
double vReal[SAMPLES];
double vImag[SAMPLES];

void i2s_install() {
  // Set up I2S Processor configuration
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 4 * 44100,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = bufferLen,
    .use_apll = false
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin() {
  // Set I2S pin configuration
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  WiFi.setHostname(hostname.c_str()); //define hostname
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void sendPushNotification() {
  initWiFi();

  // Pushover po = Pushover( ... );
	// po.setDevice( ... );
  // po.setTitle( ... );
	// po.setMessage( ... );
	// po.send(); //should return 1 on success

  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
}

void setup() {
  WiFi.mode(WIFI_OFF);

  // Set up Serial Monitor
  Serial.begin(115200);
  Serial.println(" ");
  delay(1000);

  // Set up I2S
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);

  // Set up LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  delay(500);
}

void loop() {
  // Get I2S data and place in data buffer
  size_t bytesIn = 0;
  esp_err_t result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);

  if (result == ESP_OK) {
    // Read I2S data buffer
    int16_t samples_read = bytesIn / 8;
    if (samples_read > 0) {
      for (int16_t i = 0; i < samples_read; ++i) {
        vReal[current_sample] = (sBuffer[i]);
        vImag[current_sample] = 0;
        current_sample++;

        if (current_sample >= SAMPLES) {
          current_sample = 0;

          FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weight data */
          FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);                 /* Compute FFT */
          FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);                   /* Compute magnitudes */
          double peak_frequency = FFT.MajorPeak(vReal, SAMPLES, samplingFrequency);

          if (peak_frequency > (stable_frequency * 0.95) && peak_frequency < (stable_frequency * 1.05)) {
            // (Almost) the same frequency

            if ((millis() - frequency_stable_start) > 250) {
              is_stable = true;
              digitalWrite(LED_BUILTIN, HIGH);
            }

          } else {
            // Different frequency

            // If the frequency was stable before, then print how long
            if (is_stable == true) {
              Serial.print(stable_frequency);
              Serial.print(" ");
              Serial.println(millis() - frequency_stable_start);

              // Send push notification
              sendPushNotification();

              // Take a break to make sure that not several push
              // notifications are sent at once, as the beep code
              // might be immediately repeated.
              delay(60*1000);
            }

            stable_frequency = peak_frequency;
            is_stable = false;
            frequency_stable_start = millis();
            digitalWrite(LED_BUILTIN, LOW);
          }

          // Uncomment the following lines if you like to see the signal in the
          // Serial Plotter of the Arduino IDE
          /*
          Serial.print(1000);
          Serial.print(" ");
          Serial.print(stable_frequency, 6);
          Serial.print(" ");
          Serial.print(peak_frequency, 6);  //Print out what frequency is the most dominant.
          Serial.print(" ");
          if (is_stable) {
            Serial.println(500);
          } else {
            Serial.println(0);
          }
          */
        }
      }
    }
  }
}
