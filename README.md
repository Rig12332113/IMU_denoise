# IMU Denoise (Haar Wavelet) — TCP Streaming + Live Visualization

This project receives **real-time IMU data** over **TCP**, displays the **incoming (raw) IMU signals**, and applies **Haar wavelet denoising** to show the **denoised signals** side-by-side.

- **TCP Port:** `8888`
- **Denoising method:** Haar wavelet denoise
- **Visualization:** Raw vs. Denoised IMU streams

---

## Features

- Connects to a TCP IMU stream on **port 8888**
- Parses incoming IMU samples (accelerometer / gyroscope, etc.)
- Live plot of:
  - **Raw IMU data**
  - **Haar wavelet denoised IMU data**
- Designed for real-time monitoring + quick validation of denoise quality

---

## Data Format (TCP Payload)

The server expects IMU samples to arrive as a continuous TCP stream.
```
double timestamp{};
double quat[4]{};
double acc_g[3]{};
```

## Demo
![Screenshot 2026-02-17 at 5 03 16 PM](https://github.com/user-attachments/assets/1931c1df-7797-4f90-a3c9-08c157b2565b)



