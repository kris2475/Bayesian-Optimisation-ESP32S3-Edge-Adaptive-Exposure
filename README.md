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
### Laplacian Variance

In engineering terms, the **Laplacian Variance** is a "Sharpness Scorer". It acts as a mathematical sensor that tells you how much high-frequency information (detail) is present in a signal. For the **BO-S3** project, this is the most critical part: it is the **Objective Function** that the Bayesian loop is trying to maximise.

---

#### 1. The Laplacian Operator ($\nabla^2$)
Think of the Laplacian as a **Second-Order Derivative** in 2D space. 

* **1st Derivative (Gradient):** Measures the *rate of change* (how quickly brightness changes).
* **2nd Derivative (Laplacian):** Measures the *rate of the rate of change* (how "sharp" or "abrupt" that change is).

In an image, an edge (like the rim of a coffee cup) is a sudden change in pixel intensity. The Laplacian filter highlights these edges. In a blurry or poorly exposed image, these transitions are "soft" or non-existent, so the Laplacian response is weak.



---

#### 2. The Convolution Kernel
On the ESP32, we do not perform complex calculus. We use a **Discrete Convolution Kernel**. It is a 3x3 matrix that slides over every pixel:

$$\begin{bmatrix} 
0 & 1 & 0 \\
1 & -4 & 1 \\
0 & 1 & 0 
\end{bmatrix}$$

**How it works at the pixel level:**
For every pixel, the ESP32-S3 examines its four immediate neighbours (Up, Down, Left, Right):
1.  It adds the values of the neighbours together.
2.  It subtracts 4 times the value of the centre pixel.
3.  **The Result:** If the area is flat (all pixels are the same), the result is **0**. If there is a sharp edge, the result is a high positive or negative number.

---

#### 3. Why the Variance?
The output of the Laplacian filter is a new "edge map" image. However, the Bayesian Optimiser requires a single number (a Scalar) to determine if the photo is good or bad. We calculate the **Variance** ($\sigma^2$) of this edge map to measure the "spread" of the data.



* **Low Variance (Blurry/Badly Exposed):** Most pixels in the edge map are close to zero. There is no contrast and the statistical "spread" is tiny.
* **High Variance (Sharp/Well Exposed):** The edge map contains a wide range of values—some very dark, some very bright—representing many crisp edges and high contrast.

---

#### 4. Why this is the "Engineer's Choice"
This metric is chosen because it is both **Robust** and **Cheap**:

* **Computationally Efficient:** It only requires basic addition and multiplication. It avoids square roots and trigonometry, making it perfect for the ESP32-S3's architecture.
* **Illumination Invariant-ish:** Because it is a derivative, it ignores the "DC Offset" (global brightness) and focuses solely on the "AC Component" (the edges).
* **Single Peak (Unimodal):** For a given scene, as you vary exposure from dark to light, the Laplacian Variance typically follows a "Bell Curve". This makes it exceptionally easy for Bayesian Optimisation to locate the global peak.


---

## Getting Started

1.  **Library Dependencies:**
    * `esp_camera` (Standard Espressif library)
    * `BayesianOptimization` (C++ implementation for Arduino/ESP32)
2.  **Configuration:**
    * Set `PIXFORMAT_GRAYSCALE` for maximum processing throughput.
    * Allocate the Bayesian object using `ps_malloc` to ensure it resides in PSRAM.
