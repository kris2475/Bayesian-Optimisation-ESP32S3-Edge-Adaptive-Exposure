# Bayesian-Optimisation-ESP32S3-Edge-Adaptive-Exposure

**BO-S3** is an on-device, autonomous camera tuning system developed for the **Seeed Studio XIAO ESP32-S3 Sense**. It replaces standard "average brightness" auto-exposure algorithms with a **Bayesian Optimisation (BO)** loop that maximises image detail (sharpness) rather than simple luminance.

By treating the camera registers as a "Black Box" function, the system intelligently explores the relationship between Exposure and Gain to find the global maximum of image quality in fewer than 20 iterations.

---

## Technical Overview

Traditional auto-exposure logic often fails in high-dynamic-range (HDR) or backlit conditions because it targets a mid-grey average. **BO-S3** solves this by:

1.  **Defining a Quality Metric:** Utilising the **Laplacian Variance** of the image buffer to quantify edge density and detail.
2.  **Global Search:** Implementing a **Gaussian Process (GP)** surrogate model to map the 2D parameter space (Exposure vs. Gain).
3.  **Smart Sampling:** Employing the **Upper Confidence Bound (UCB)** acquisition function to balance the exploration of unknown settings with the exploitation of known high-performing settings.

---

## Hardware Requirements

* **Microcontroller:** Seeed Studio XIAO ESP32-S3 Sense
* **Sensor:** OV2640 Camera Module (included)
* **Memory:** 8MB PSRAM (Crucial for storing the GP covariance matrix and frame buffers)

---

## System Architecture

The project operates in a closed-loop "Observe-Think-Act" cycle entirely on the ESP32-S3 silicon:

* **ACT:** Manipulate manual camera registers (`aec_value` for exposure, `agc_value` for gain).
* **OBSERVE:** Capture a grayscale QVGA frame and calculate the Laplacian Variance.
* **THINK:** Update the Gaussian Process. The system calculates the next optimal sample point based on the **Expected Improvement**.



---

## Key Features

* **Pure Edge Implementation:** No cloud, no Wi-Fi, and no external PC required for the linear algebra.
* **PSRAM Optimised:** Leverages external RAM to handle larger matrices, improving the precision and depth of the Gaussian Process.
* **Hardware Acceleration:** Utilises the ESP32-S3's dual-core LX7 processor to manage concurrent image processing and matrix inversion.
* **Utilitarian Design:** Engineered for efficiency, reminiscent of high-stakes "one-shot" environments where every sample must be meaningful.

---

## Objective Function: Laplacian Variance

The system maximises the variance of the Laplacian operator $\nabla^2 I$:

$$y = \text{Var}(\nabla^2 I) = E[(\nabla^2 I)^2] - (E[\nabla^2 I])^2$$

A higher value indicates a sharp, well-exposed image with high detail. A lower value indicates an image that is either "crushed" (underexposed), "blown out" (overexposed), or out of focus.



---

## Getting Started

1.  **Library Dependencies:**
    * `esp_camera` (Standard Espressif library)
    * `BayesianOptimization` (C++ implementation for Arduino/ESP32)
2.  **Configuration:**
    * Set `PIXFORMAT_GRAYSCALE` for maximum processing throughput.
    * Allocate the Bayesian object using `ps_malloc` to ensure it resides in PSRAM.
