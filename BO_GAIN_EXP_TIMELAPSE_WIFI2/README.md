# Adaptive Exposure & Gain Timelapse (XIAO ESP32-S3 Sense)

### `BO_Adaptive_Exposure_Gain_Timelapse_Persistent_NTP_FIXED.ino`

---

## ⚠️ IMPORTANT: NOT FORMAL BAYESIAN OPTIMIZATION (BO)
This project implements a **"Heuristic BO-Inspired"** acquisition loop. Please note that this is **NOT** a formal statistical Bayesian Optimization framework. It lacks:
* Gaussian Process (GP) regression.
* Cholesky decomposition.
* Formal marginal likelihood maximization.

Instead, it utilizes a **simplified Radial Basis Function (RBF) kernel** approximation to estimate mean and variance for an **Upper Confidence Bound (UCB)** decision-making process. It is a lightweight, edge-computing approximation designed for real-time adjustments on low-power hardware.

---

## 🚀 Version: Fully Corrected & Stabilized
This version addresses critical stability and hardware-specific issues identified in previous iterations:

* **Explicit SPI SD Initialization:** Specifically defined for the **XIAO ESP32-S3 Sense** expansion board (SCK, MISO, MOSI, CS).
* **PGM Compliance:** Properly prepends the `P5` header (Magic Number, Dimensions, MaxVal) to the raw buffer, ensuring files are immediately viewable in standard image viewers.
* **Sensor Mapping:** Correctly maps internal float gain values (0.0–5.0) to the **OV2640** integer register domain (0–30).
* **Safety Constraints:** Implements strict `constrain()` calls on all parameters before hardware application to prevent sensor crashes.

---

## 🧠 Computational Methodology

The system treats the camera settings as an optimization problem over a 2D search space:

| Component | Method |
| :--- | :--- |
| **Search Space** | 2D Grid Search (Exposure $\times$ Gain) |
| **Objective Function** | Edge Detection (pixel-to-pixel delta) minus Brightness Deviation penalty |
| **Kernel** | RBF Kernel: $k(x, x') = \exp\left(-\frac{(x - x')^2}{2\ell^2}\right)$ |
| **Acquisition Strategy** | **UCB (Upper Confidence Bound):** $\text{Score} = \mu + (\kappa \cdot \sigma)$ |

---

## 🛠️ Core Logic & Features

1.  **Adaptive Automation:** Automatically adjusts exposure and gain to maximize image "quality" (defined as high edge-contrast and adherence to target-mean brightness).
2.  **State Persistence:** Saves the "BO" state (historical samples and scores) to the microSD card as `/bo_state.dat`. This allows the algorithm to resume its learning progress after reboots or power cycles.
3.  **NTP Time Synchronization:** Connects to WiFi once at boot to synchronize time via NTP, enabling human-readable **ISO-8601** filenames (e.g., `20260219_185600.pgm`).
4.  **Hardware Optimization:** Specifically tuned for the pinout and memory constraints of the **Seeed Studio XIAO ESP32-S3 Sense**.

---

## 📁 File Structure
* **SD:/**
    * `bo_state.dat`: Binary file containing the optimization history.
    * `YYYYMMDD_HHMMSS.pgm`: Grayscale timelapse frames.

---

**Last Updated:** February 19, 2026  
**Status:** Stable / Production Ready for XIAO ESP32-S3

Would you like me to generate a **Python script** to help you convert these `.pgm` files into a `.mp4` timelapse video?
