# Bayesian-Optimisation-ESP32S3-Edge-Adaptive-Exposure
BO-S3: Edge Adaptive Exposure uses Bayesian Optimisation to automate camera tuning on the ESP32-S3. By treating exposure and gain as a "black box," the system avoids slow grid searches. It captures frames, calculates Laplacian Variance to measure image sharpness, and uses a Gaussian Process to predict the ideal settings in ~15 iterations.
