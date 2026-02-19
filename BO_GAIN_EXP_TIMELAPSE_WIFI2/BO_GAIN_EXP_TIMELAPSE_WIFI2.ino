/**************************************************************************************
 * PROJECT: Adaptive Exposure & Gain Timelapse (XIAO ESP32-S3 Sense)
 * FILE:    BO_Adaptive_Exposure_Gain_Timelapse_Persistent_NTP_FIXED.ino
 * * ⚠️ IMPORTANT: NOT FORMAL BAYESIAN OPTIMIZATION (BO) ⚠️
 * This script implements a "Heuristic BO-Inspired" acquisition loop. It is NOT a 
 * formal statistical Bayesian Optimization framework (e.g., no Gaussian Process 
 * regression, no Cholesky decomposition, and no formal marginal likelihood 
 * maximization). It uses a simplified Radial Basis Function (RBF) kernel 
 * approximation to estimate mean and variance for an Upper Confidence Bound (UCB) 
 * decision-making process.
 * * VERSION: FULLY CORRECTED & STABILIZED
 * * SUMMARY OF CORE LOGIC:
 * 1.  INTENT: Automate camera exposure and gain settings to maximize image 
 * "quality" (defined as high edge-contrast and target-mean brightness).
 * 2.  PERSISTENCE: Saves the "BO" state (historical samples and scores) to 
 * microSD (/bo_state.dat) to survive reboots/power cycles.
 * 3.  TIME: Synchronizes via NTP once at boot to provide human-readable 
 * ISO-8601 filenames.
 * 4.  HARDWARE: Specifically tuned for the XIAO ESP32-S3 Sense (SPI SD config).
 * * KEY FIXES APPLIED:
 * - SPI INITIALIZATION: Explicitly defines SCK/MISO/MOSI/CS for the XIAO expansion board.
 * - PGM COMPLIANCE: Prepends the necessary P5 header (Magic Number, Dim, MaxVal) 
 * to the raw buffer.
 * - SENSOR MAPPING: Maps float gain (0.0-5.0) to OV2640 register domain (0-30).
 * - SAFETY: Applies strict constrain() calls to prevent invalid register writes.
 * * COMPUTATIONAL METHODOLOGY:
 * - Search Space: 2D Grid Search (Exposure x Gain).
 * - Objective Function: Edge detection (pixel-to-pixel delta) minus a brightness 
 * deviation penalty.
 * - Acquisition: Upper Confidence Bound (UCB) where Score = Mean + (Kappa * StdDev).
 * * DATE: February 19, 2026
 **************************************************************************************/

#include "esp_camera.h"
#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include <WiFi.h>
#include <time.h>
#include <math.h>

/*========================= NETWORK =========================*/
#define WIFI_SSID  " "
#define WIFI_PASS  " "

/*========================= SPI SD ==========================*/
#define SD_CS   21
#define SD_SCK  7
#define SD_MISO 8
#define SD_MOSI 9

/*========================= CAMERA PINS =====================*/
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

/*========================= TIMELAPSE =======================*/
const unsigned long TIMELAPSE_INTERVAL = 60000;
unsigned long lastCapture = 0;

/*========================= BO PARAMETERS ===================*/
const int MAX_SAMPLES = 25;
const int FORCED_RANDOM_SAMPLES = 8;

const float LENGTH_SCALE = 150.0f;
const float KAPPA = 2.0f;

const float MIN_EXPOSURE = 20.0f;
const float MAX_EXPOSURE = 1200.0f;
const float MIN_GAIN = 0.0f;
const float MAX_GAIN = 5.0f;

const float EXPOSURE_STEP = 40.0f;
const float GAIN_STEP = 0.5f;

const float TARGET_BRIGHTNESS = 110.0f;
const float BRIGHTNESS_WEIGHT = 0.02f;

struct Sample {
  float exposure;
  float gain;
  float score;
};

Sample obs[MAX_SAMPLES];
int sample_idx = 0;

/*========================= KERNEL ==========================*/
float kernel(float x1, float x2) {
  float d = x1 - x2;
  return expf(-(d * d) / (2.0f * LENGTH_SCALE * LENGTH_SCALE));
}

/*========================= BO ==============================*/
Sample calculate_next_settings() {

  if (sample_idx < FORCED_RANDOM_SAMPLES) {
    Sample s;
    s.exposure = random(MIN_EXPOSURE, MAX_EXPOSURE);
    s.gain = random(MIN_GAIN * 10, MAX_GAIN * 10) / 10.0f;
    return s;
  }

  float best_ucb = -1e9f;
  Sample best = {MIN_EXPOSURE, MIN_GAIN, 0};

  for (float e = MIN_EXPOSURE; e <= MAX_EXPOSURE; e += EXPOSURE_STEP) {
    for (float g = MIN_GAIN; g <= MAX_GAIN; g += GAIN_STEP) {

      float mean = 0.0f;
      float var = 0.1f;

      for (int i = 0; i < sample_idx; i++) {
        float ke = kernel(e, obs[i].exposure);
        float kg = kernel(g, obs[i].gain);
        float k = ke * kg;
        mean += k * obs[i].score;
        var += (1.0f - k);
      }

      float ucb = mean + KAPPA * sqrtf(var);

      if (ucb > best_ucb) {
        best_ucb = ucb;
        best.exposure = e;
        best.gain = g;
      }
    }
  }

  return best;
}

/*========================= SCORE ===========================*/
float compute_score(camera_fb_t *fb) {

  uint32_t edge_sum = 0;
  uint32_t bright_sum = 0;

  for (size_t i = 0; i < fb->len - 1; i++) {
    uint8_t p = fb->buf[i];
    uint8_t n = fb->buf[i + 1];
    edge_sum += abs((int)p - (int)n);
    bright_sum += p;
  }

  float edge = (float)edge_sum / fb->len;
  float mean_brightness = (float)bright_sum / fb->len;

  float penalty = BRIGHTNESS_WEIGHT *
                  abs(mean_brightness - TARGET_BRIGHTNESS);

  return edge - penalty;
}

/*========================= PERSISTENCE =====================*/
void save_bo_state() {
  File f = SD.open("/bo_state.dat", FILE_WRITE);
  if (!f) return;
  f.write((uint8_t*)&sample_idx, sizeof(sample_idx));
  f.write((uint8_t*)obs, sizeof(obs));
  f.close();
}

void load_bo_state() {
  if (!SD.exists("/bo_state.dat")) return;
  File f = SD.open("/bo_state.dat");
  if (!f) return;
  f.read((uint8_t*)&sample_idx, sizeof(sample_idx));
  f.read((uint8_t*)obs, sizeof(obs));
  f.close();
}

/*========================= SD ==============================*/
void setup_sd() {
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD mount failed.");
    while (true);
  }
  Serial.println("SD Card ready (SPI)");
}

/*========================= CAMERA ==========================*/
void setup_camera() {

  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.frame_size = FRAMESIZE_QVGA;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed.");
    while (true);
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_exposure_ctrl(s, 0);
  s->set_gain_ctrl(s, 0);

  Serial.println("Camera OK");
}

/*========================= TIME ============================*/
void setup_time() {

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  configTime(0, 0, "pool.ntp.org");

  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) delay(500);

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  Serial.println("Time synchronized.");
}

/*========================= SETUP ===========================*/
void setup() {

  Serial.begin(115200);
  delay(2000);

  Serial.println("\n--- Bayesian Adaptive Exposure + Gain Timelapse ---");

  setup_sd();
  load_bo_state();
  setup_camera();
  setup_time();

  Serial.println("System Ready.");
}

/*========================= LOOP ============================*/
void loop() {

  if (millis() - lastCapture < TIMELAPSE_INTERVAL)
    return;

  lastCapture = millis();

  Sample next = calculate_next_settings();

  next.exposure = constrain(next.exposure, MIN_EXPOSURE, MAX_EXPOSURE);
  next.gain = constrain(next.gain, MIN_GAIN, MAX_GAIN);

  sensor_t *s = esp_camera_sensor_get();
  s->set_aec_value(s, (int)next.exposure);

  int sensor_gain = (int)(next.gain * 6.0f);  // map 0–5 → approx 0–30
  sensor_gain = constrain(sensor_gain, 0, 30);
  s->set_agc_gain(s, sensor_gain);

  delay(250);

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;

  float score = compute_score(fb);

  obs[sample_idx] = {next.exposure, next.gain, score};
  sample_idx = (sample_idx + 1) % MAX_SAMPLES;

  save_bo_state();

  struct tm timeinfo;
  getLocalTime(&timeinfo);

  char filename[32];
  strftime(filename, sizeof(filename), "/%Y%m%d_%H%M%S.pgm", &timeinfo);

  File file = SD.open(filename, FILE_WRITE);
  if (file) {
    file.printf("P5\n%d %d\n255\n", fb->width, fb->height);
    file.write(fb->buf, fb->len);
    file.close();
  }

  esp_camera_fb_return(fb);

  Serial.printf("Iter: %02d | Exposure: %.1f | Gain: %.2f | Score: %.2f\n",
                sample_idx, next.exposure, next.gain, score);
}


















