/**
 * @file BO-S3_Edge_Adaptive_Exposure.ino
 * @project BO-S3: Edge Adaptive Exposure
 * @date 2026-02-17
 * * DESCRIPTION:
 * This sketch implements an on-device Bayesian Optimisation (BO) loop to automate
 * camera exposure and gain tuning. Unlike standard "Auto" modes, it maximises
 * "Image Detail" (Laplacian Variance) rather than average brightness.
 * * ENGINEERING CONTEXT:
 * Developed for the Seeed Studio XIAO ESP32-S3 Sense.
 * Uses 8MB PSRAM for the Gaussian Process matrix and frame buffers.
 * Objective Function: Statistical Variance of the 2D Discrete Laplacian.
 * * DESIGNED FOR: 
 * High-stakes "one-shot" scenarios, where the system 
 * must find the global maximum of a black-box function in minimum iterations.
 */

#include "esp_camera.h"
#include <BayesianOptimization.h>

// --- Camera Configuration ---
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

// --- Project Constants ---
const int MAX_ITERATIONS = 20; // Limit due to O(N^3) matrix inversion complexity
int current_iter = 0;

// Bayesian Space: Dim 0 = Exposure (aec_value), Dim 1 = Gain (agc_value)
float min_bounds[] = {10.0, 0.0};
float max_bounds[] = {1200.0, 30.0};
BayesianOptimization bo(2); // 2-dimensional optimisation

void setup() {
  Serial.begin(115200);
  
  // 1. Initialise Camera
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA; // 320x240 for fast processing
  config.pixel_format = PIXFORMAT_GRAYSCALE; // Direct luminance access
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM; // Use XIAO's PSRAM
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera Init Failed");
    while (true);
  }

  // 2. Disable Hardware Auto-Exposure/Gain to hand control to BO
  sensor_t * s = esp_camera_sensor_get();
  s->set_aec_mode(s, 0); // Manual Exposure
  s->set_agc_mode(s, 0); // Manual Gain

  // 3. Initialise Bayesian Bounds
  bo.setBounds(min_bounds, max_bounds);
  
  Serial.println("BO-S3 System Ready. Commencing Optimisation Loop...");
}

void loop() {
  if (current_iter < MAX_ITERATIONS) {
    
    // Step A: "Think" - Request next optimal test points from Bayesian Engine
    float* next_guess = bo.propose();
    int exposure = (int)next_guess[0];
    int gain = (int)next_guess[1];

    // Step B: "Act" - Apply settings to the OV2640 sensor
    sensor_t * s = esp_camera_sensor_get();
    s->set_aec_value(s, exposure);
    s->set_agc_value(s, gain);
    
    // Allow sensor to settle (CMOS charge time)
    delay(500);

    // Step C: "Observe" - Capture frame and evaluate quality
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Frame Capture Failed");
      return;
    }

    float sharpness_score = calculate_laplacian_variance(fb->buf, fb->width, fb->height);
    
    // Step D: "Learn" - Feed result back into Gaussian Process
    bo.update(next_guess, sharpness_score);
    
    // Clean up frame buffer
    esp_camera_fb_return(fb);

    // Progress Output
    Serial.printf("Iter: %d | Exp: %d | Gain: %d | Score: %.2f\n", current_iter, exposure, gain, sharpness_score);
    
    current_iter++;
  } else {
    // Optimisation Complete
    float* best_params = bo.getBestX();
    Serial.println("--- OPTIMISATION COMPLETE ---");
    Serial.printf("Optimal Exposure: %d\n", (int)best_params[0]);
    Serial.printf("Optimal Gain: %d\n", (int)best_params[1]);
    
    // Stay at best settings
    sensor_t * s = esp_camera_sensor_get();
    s->set_aec_value(s, (int)best_params[0]);
    s->set_agc_value(s, (int)best_params[1]);
    
    while(1); // Stop
  }
}

/**
 * CALCULATE LAPLACIAN VARIANCE
 * ----------------------------
 * Computes the statistical variance of the 2D Laplacian.
 * Acts as a proxy for high-frequency detail (sharpness).
 */
float calculate_laplacian_variance(uint8_t* buffer, int width, int height) {
  long sum = 0;
  long sq_sum = 0;
  int count = 0;

  for (int y = 1; y < height - 1; y++) {
    for (int x = 1; x < width - 1; x++) {
      // Discrete Convolution Kernel: [0, 1, 0][1, -4, 1][0, 1, 0]
      int center = buffer[y * width + x];
      int neighbors = buffer[(y - 1) * width + x] + 
                      buffer[(y + 1) * width + x] + 
                      buffer[y * width + (x - 1)] + 
                      buffer[y * width + (x + 1)];
      
      int laplacian = neighbors - (4 * center);
      
      sum += laplacian;
      sq_sum += (long)laplacian * laplacian;
      count++;
    }
  }

  // Engineering formula for Variance: E[X^2] - (E[X])^2
  float mean = (float)sum / count;
  float variance = ((float)sq_sum / count) - (mean * mean);
  
  return variance;
}
