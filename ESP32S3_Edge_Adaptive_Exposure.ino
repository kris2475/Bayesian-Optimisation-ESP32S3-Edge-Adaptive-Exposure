/**************************************************************************************
 * NOTE: BO INSPIRED
 *
 *  BO_Adaptive_Exposure_Timelapse_Grayscale.ino
 *
 *  Target Hardware:
 *      Seeed Studio XIAO ESP32S3 + Sense Shield (SPI SD slot)
 *
 *  Software Environment:
 *      Arduino ESP32 Core 2.0.17
 *      Built-in esp32-camera driver (ESP-IDF 4.4.x based)
 *
 *  -----------------------------------------------------------------------------------
 *  PURPOSE
 *  -----------------------------------------------------------------------------------
 *  Stable timelapse capture of plants with **Bayesian Optimised exposure** for maximum sharpness.
 *  Uses **SPI SD (Sense Shield slot)** and **grayscale QVGA** to avoid DMA/PSRAM conflicts.
 *
 *  - Camera initialised once at setup
 *  - SD initialised once at setup
 *  - Frames saved as raw grayscale .pgm (lightweight)
 *  - Edge-density scoring feeds Bayesian Optimisation for each frame
 *  - Timelapse interval: 1 minute
 *
 *  -----------------------------------------------------------------------------------
 *  FEATURES
 *  -----------------------------------------------------------------------------------
 *      • Disables auto exposure control
 *      • Bayesian Optimisation using UCB
 *      • Grayscale QVGA frames
 *      • Edge-density scoring (1D gradient)
 *      • Timelapse-ready SD saving (SPI)
 *      • Fixed memory footprint
 *
 *  -----------------------------------------------------------------------------------
 *  SETTINGS
 *  -----------------------------------------------------------------------------------
 *      • MAX_SAMPLES       : Max BO observations
 *      • LENGTH_SCALE      : Kernel length scale for BO
 *      • KAPPA             : UCB exploration factor
 *      • MIN_EXPOSURE      : Min camera exposure
 *      • MAX_EXPOSURE      : Max camera exposure
 *      • EXPOSURE_STEP     : Step size for candidate exposures
 *      • TIMELAPSE_INTERVAL: Interval between frames (ms)
 *
 **************************************************************************************/

#include "esp_camera.h"
#include "Arduino.h"
#include "SD.h"
#include <math.h>

/*==============================================================================*/
/*  Camera Pin Definitions (XIAO ESP32S3)                                      */
/*==============================================================================*/
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

/*==============================================================================*/
/*  SPI SD Pin Definitions (Sense Shield)                                      */
/*==============================================================================*/
#define SD_CS   21
#define SD_SCK  7
#define SD_MISO 8
#define SD_MOSI 9

/*==============================================================================*/
/*  Bayesian Optimization Parameters                                           */
/*==============================================================================*/
static const int MAX_SAMPLES      = 25;
static const float LENGTH_SCALE   = 150.0f;
static const float KAPPA          = 2.0f;
static const float MIN_EXPOSURE   = 20.0f;
static const float MAX_EXPOSURE   = 1200.0f;
static const float EXPOSURE_STEP  = 20.0f;

float obs_exposure[MAX_SAMPLES];
float obs_score[MAX_SAMPLES];
int total_samples = 0;

/*==============================================================================*/
/*  Timelapse Interval                                                          */
/*==============================================================================*/
static const unsigned long TIMELAPSE_INTERVAL = 60000; // 1 minute
unsigned long lastCapture = 0;

/*==============================================================================*/
/*  Squared Exponential Kernel                                                  */
/*==============================================================================*/
float kernel(float x1, float x2){
    float diff = x1 - x2;
    return expf(-(diff*diff)/(2.0f*LENGTH_SCALE*LENGTH_SCALE));
}

/*==============================================================================*/
/*  Upper Confidence Bound Acquisition                                          */
/*==============================================================================*/
float calculate_next_exposure(){
    if(total_samples==0) return 100.0f; // initial guess

    float best_x = MIN_EXPOSURE;
    float max_ucb = -1e9f;

    for(float x=MIN_EXPOSURE;x<=MAX_EXPOSURE;x+=EXPOSURE_STEP){
        float mean=0.0f,var=1.0f;
        for(int i=0;i<total_samples;i++){
            float k = kernel(x, obs_exposure[i]);
            mean += k*obs_score[i];
            var  -= k*k;
        }
        float variance = (var>0.1f)?var:0.1f;
        float ucb = mean + KAPPA*sqrtf(variance);
        if(ucb>max_ucb){
            max_ucb=ucb;
            best_x=x;
        }
    }
    return best_x;
}

/*==============================================================================*/
/*  Camera Initialization                                                       */
/*==============================================================================*/
bool setup_camera(){
    camera_config_t config;

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;

    config.pin_d0 = Y2_GPIO_NUM; config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM; config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM; config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM; config.pin_d7 = Y9_GPIO_NUM;

    config.pin_xclk  = XCLK_GPIO_NUM;
    config.pin_pclk  = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href  = HREF_GPIO_NUM;

    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;

    config.pin_pwdn  = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;

    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE; // lighter than JPEG
    config.frame_size   = FRAMESIZE_QVGA;
    config.fb_count     = 1; // single framebuffer for stability
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if(err != ESP_OK){
        Serial.printf("Camera init failed: 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_exposure_ctrl(s,0); // disable auto exposure

    return true;
}

/*==============================================================================*/
/*  SPI SD Initialization                                                      */
/*==============================================================================*/
bool setup_sd(){
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    if(!SD.begin(SD_CS)){
        Serial.println("SD Card Mount Failed (SPI)");
        return false;
    }
    delay(50);
    return true;
}

/*==============================================================================*/
/*  Edge-Density Scoring                                                        */
/*==============================================================================*/
float compute_edge_density(camera_fb_t *fb){
    uint32_t edge_sum = 0;
    for(size_t i=0;i<fb->len-1;i++)
        edge_sum += abs((int)fb->buf[i] - (int)fb->buf[i+1]);
    return (float)edge_sum / (float)fb->len;
}

/*==============================================================================*/
/*  Setup                                                                       */
/*==============================================================================*/
void setup(){
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n--- Bayesian Adaptive Exposure Timelapse (SPI SD) ---");

    if(!setup_sd()){
        Serial.println("ERROR: SD mount failed! Check SD card.");
        while(true);
    }
    Serial.println("SD Card ready (SPI)");

    if(!setup_camera()){
        Serial.println("ERROR: Camera init failed! Check connections.");
        while(true);
    }
    Serial.println("Camera OK");

    Serial.println("\n--- System Ready: Camera + SD verified ---");
    lastCapture = millis() - TIMELAPSE_INTERVAL; // first capture immediately
}

/*==============================================================================*/
/*  Main Loop                                                                   */
/*==============================================================================*/
void loop(){
    if(millis() - lastCapture < TIMELAPSE_INTERVAL) return;
    lastCapture = millis();

    // Determine optimal exposure
    float optimal_exposure = calculate_next_exposure();
    sensor_t *s = esp_camera_sensor_get();
    s->set_aec_value(s, round(optimal_exposure));
    delay(250); // settle

    // Capture frame
    camera_fb_t *fb = esp_camera_fb_get();
    if(!fb){
        Serial.println("Frame capture failed");
        return;
    }

    float score = compute_edge_density(fb);

    // Save as .pgm (grayscale raw)
    String filename = "/plant_" + String(millis()) + ".pgm";
    File file = SD.open(filename.c_str(), FILE_WRITE);
    if(file){
        file.write(fb->buf, fb->len);
        file.close();
        Serial.println("Saved: " + filename);
    } else {
        Serial.println("SD write failed: " + filename);
    }

    // Feed BO
    if(total_samples < MAX_SAMPLES){
        obs_exposure[total_samples] = optimal_exposure;
        obs_score[total_samples] = score;
        total_samples++;
    } else {
        for(int i=0;i<MAX_SAMPLES-1;i++){
            obs_exposure[i]=obs_exposure[i+1];
            obs_score[i]=obs_score[i+1];
        }
        obs_exposure[MAX_SAMPLES-1]=optimal_exposure;
        obs_score[MAX_SAMPLES-1]=score;
    }

    esp_camera_fb_return(fb);

    Serial.printf("Iter: %02d | Exposure: %6.1f | Edge Score: %6.2f\n",
                  total_samples, optimal_exposure, score);
}

