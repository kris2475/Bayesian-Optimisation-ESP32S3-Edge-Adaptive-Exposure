# Bayesian-Optimisation-ESP32S3-Edge-Adaptive-Exposure

**BO-S3 (Embedded Variant)** is a lightweight, on-device adaptive exposure system for the Seeed Studio XIAO ESP32-S3 Sense.

It replaces traditional brightness-based auto-exposure with a Bayesian-inspired optimisation loop that maximises image detail (edge density) rather than average luminance.

This implementation is intentionally engineered for embedded reliability, deterministic memory usage, and long-duration timelapse stability — not full academic Gaussian Process regression.

---

# What This Project Is — and Is Not

## ✅ What It Is

- A 1D Bayesian-inspired optimiser
- Optimises manual exposure (`aec_value`)
- Uses a kernel-based surrogate model
- Uses Upper Confidence Bound (UCB) acquisition
- Uses a fast edge-density sharpness metric
- Runs entirely on-device
- Designed for minute-scale timelapse stability

## ❌ What It Is Not

- Not a full Gaussian Process regression
- Not a 2D optimiser (gain is not optimised)
- No covariance matrix construction
- No matrix inversion
- No Expected Improvement (EI)
- No heavy PSRAM linear algebra
- No full Laplacian variance convolution

This is Bayesian-inspired optimisation adapted for microcontroller constraints.

---

# Why This Design Exists

The XIAO ESP32-S3 Sense is powerful for its size, but:

- PSRAM bandwidth is limited
- Frame buffer memory must remain stable
- Matrix inversion scales O(n³)
- Long-running timelapse systems must avoid heap fragmentation
- Deterministic timing is critical

A full Gaussian Process implementation with covariance matrix inversion on every iteration would:

- Increase PSRAM pressure
- Risk watchdog resets
- Increase power consumption
- Compromise long-duration stability

Instead, this system uses a resource-aware surrogate approximation that preserves the spirit of Bayesian Optimisation without the computational burden.

---

# Core Concept

Rather than maximise average brightness (like traditional auto-exposure), the system maximises edge density, a proxy for sharpness and usable detail.

The optimisation loop operates as:

ACT → OBSERVE → SCORE → UPDATE → REPEAT

Each minute:

1. Select next exposure via UCB
2. Apply manual exposure
3. Capture grayscale QVGA frame
4. Compute edge-density score
5. Update surrogate model

---

# Objective Function (Embedded Variant)

Instead of a full 3×3 Laplacian convolution and variance calculation, this implementation uses a 1D gradient magnitude approximation:


### Why This Choice?

- O(n) complexity
- No convolution kernel
- No additional frame buffer
- No floating-point heavy operations
- No square roots
- Stable on long runs
- Works well in practice for exposure tuning

While not a mathematically pure Laplacian variance, it captures the same principle:

> More edge contrast = better exposure.

---

# Surrogate Model

The optimiser uses a squared exponential kernel:

k(x1, x2) = exp( - (x1 - x2)^2 / (2l^2) )


However, this is not a full GP posterior.

Instead of computing:

μ(x) = kᵀ K⁻¹ y
σ²(x) = k(x,x) - kᵀ K⁻¹ k


This implementation uses a simplified accumulation model:

mean ≈ Σ k(x, xi) * yi
variance ≈ 1 - Σ k(x, xi)^2


This provides:

- Smooth interpolation
- Exploration capability
- Convergence behaviour
- Minimal RAM footprint
- No matrix inversion

It is an embedded approximation of GP behaviour.

---

# Why UCB Instead of Expected Improvement?

UCB was chosen because:

- No need for posterior CDF
- No Gaussian distribution assumptions
- No error function (`erf`)
- Deterministic computation
- Very stable numerically
- Efficient on embedded cores

For microcontrollers, predictability is more important than theoretical purity.

---

# Hardware Requirements

- Board: Seeed Studio XIAO ESP32-S3 Sense
- Sensor: OV2640
- Frame Size: QVGA Grayscale
- Memory: 8MB PSRAM recommended (for frame buffers)

---

# Why Not Use Standard Auto-Exposure?

Traditional auto-exposure:

- Targets mid-grey
- Fails in HDR/backlit conditions
- Optimises brightness, not detail
- Can overexpose highlights or crush shadows

This system:

- Ignores global luminance
- Maximises edge response
- Naturally avoids blown highlights
- Converges to a detail-rich exposure

For plant timelapse, structural detail matters more than brightness averages.

---

# Why This Approach Is Ideal for Timelapse

- Exposure stabilises after convergence
- Adapts slowly to lighting drift
- Deterministic 1-minute interval execution
- No cloud dependency
- No Wi-Fi required
- No PC required
- Runs indefinitely without memory growth

The design prioritises:

- Stability
- Predictability
- Embedded efficiency
- Low power operation

---

# Summary

BO-S3 (Embedded Variant) is:

A pragmatic, resource-aware Bayesian-inspired exposure optimiser designed specifically for long-duration autonomous operation on the XIAO ESP32-S3 Sense.

It is inspired by Gaussian Process Bayesian Optimisation but deliberately simplified to meet embedded engineering constraints.

This is not academic GP research software.

It is engineered firmware.

