#include "WaveletDenoiser.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

// ---------- ctor ----------
WaveletDenoiser::WaveletDenoiser(std::size_t window_size, int levels, std::size_t hop)
    : N_(window_size), L_(levels), hop_(hop) {
    // This implementation currently supports exactly N=256.
    // If you change N_, also change the fixed arrays in the header.
    if (N_ != 256) {
        N_ = 256;
    }
    if (L_ < 1) L_ = 1;
    if (L_ > 8) L_ = 8; // safe cap for 256
    if (hop_ < 1) hop_ = 1;
}

// ---------- public ----------
bool WaveletDenoiser::push(double t, double ax, double ay, double az) {
    t_[write_idx_]  = t;
    ax_[write_idx_] = ax;
    ay_[write_idx_] = ay;
    az_[write_idx_] = az;

    write_idx_ = (write_idx_ + 1) % N_;
    if (filled_ < N_) filled_++;
    since_last_++;

    // Only produce output when we have a full window and hop reached.
    if (filled_ == N_ && since_last_ >= hop_) {
        since_last_ = 0;
        run_denoise_and_emit();
        return true;
    }
    return false;
}

std::optional<DenoisedAccel> WaveletDenoiser::latest() const {
    return latest_;
}

// ---------- internal ----------
void WaveletDenoiser::run_denoise_and_emit() {
    // Reconstruct the last N samples in time order into a contiguous window.
    // Oldest is at write_idx_ (because write_idx_ points to next write position).
    std::array<double, 256> wx{}, wy{}, wz{};
    std::array<double, 256> wt{};

    std::size_t idx = write_idx_;
    for (std::size_t i = 0; i < N_; ++i) {
        wt[i] = t_[idx];
        wx[i] = ax_[idx];
        wy[i] = ay_[idx];
        wz[i] = az_[idx];
        idx = (idx + 1) % N_;
    }

    // Keep raw newest sample (last element in time-ordered window)
    DenoisedAccel out;
    out.t = wt[N_ - 1];
    out.raw = {wx[N_ - 1], wy[N_ - 1], wz[N_ - 1]};

    auto denoise_axis = [&](std::array<double, 256>& w) -> double {
        // Forward DWT
        haar_dwt_inplace(w, L_);

        // Noise estimate from highest-frequency detail band at level 1:
        // In Haar, after full transform, the last N/2 coefficients are the finest detail band (D1).
        // For robustness, use MAD on that band.
        const std::size_t d1_start = N_ / 2;
        const std::size_t d1_len   = N_ / 2;
        double sigma = mad_sigma_from_detail(w, d1_start, d1_len);

        // Universal threshold
        double T = sigma * std::sqrt(2.0 * std::log(static_cast<double>(N_)));

        // Soft-threshold all detail coefficients (exclude the top approximation block).
        // After L levels, approximation block length is N / 2^L at the beginning.
        std::size_t approx_len = N_;
        for (int i = 0; i < L_; ++i) approx_len /= 2;

        soft_threshold_range(w, approx_len, N_ - approx_len, T);

        // Inverse DWT
        haar_idwt_inplace(w, L_);

        // Return newest denoised value
        return w[N_ - 1];
    };

    // Denoise each axis on a copy of the window
    double dx = denoise_axis(wx);
    double dy = denoise_axis(wy);
    double dz = denoise_axis(wz);

    out.den = {dx, dy, dz};
    latest_ = out;
}

// ---------- Haar DWT/IDWT (in-place, array size 256) ----------
void WaveletDenoiser::haar_dwt_inplace(std::array<double, 256>& x, int levels) {
    // In-place iterative Haar:
    // For each level, transform the first "len" samples into approx+detail (len/2 each)
    // stored back into x[0:len).
    std::array<double, 256> temp{};

    std::size_t len = 256;
    for (int l = 0; l < levels; ++l) {
        std::size_t half = len / 2;
        const double inv_sqrt2 = 1.0 / std::sqrt(2.0);

        for (std::size_t i = 0; i < half; ++i) {
            double a = x[2 * i];
            double b = x[2 * i + 1];
            temp[i]       = (a + b) * inv_sqrt2; // approximation
            temp[half+i]  = (a - b) * inv_sqrt2; // detail
        }
        // Copy back only the first len values
        for (std::size_t i = 0; i < len; ++i) x[i] = temp[i];

        len = half;
        if (len < 2) break;
    }
}

void WaveletDenoiser::haar_idwt_inplace(std::array<double, 256>& x, int levels) {
    // Inverse of above; reconstruct from smallest approx upwards.
    std::array<double, 256> temp{};

    // Starting length of approximation block after forward transform
    std::size_t len = 256;
    for (int i = 0; i < levels; ++i) len /= 2;
    if (len < 1) len = 1;

    // Iteratively reconstruct: at each step, current approx length = len, detail length = len,
    // combined output length = 2*len.
    for (int l = 0; l < levels; ++l) {
        std::size_t half = len;
        std::size_t full = 2 * len;
        const double inv_sqrt2 = 1.0 / std::sqrt(2.0);

        for (std::size_t i = 0; i < half; ++i) {
            double a = x[i];
            double d = x[half + i];
            temp[2 * i]     = (a + d) * inv_sqrt2;
            temp[2 * i + 1] = (a - d) * inv_sqrt2;
        }
        for (std::size_t i = 0; i < full; ++i) x[i] = temp[i];

        len = full;
        if (len >= 256) break;
    }
}

// ---------- robust stats ----------
double WaveletDenoiser::median(std::array<double, 256> v, std::size_t n) {
    // Compute median of first n entries in v (copy by value)
    if (n == 0) return 0.0;
    auto begin = v.begin();
    auto mid = begin + static_cast<std::ptrdiff_t>(n / 2);
    std::nth_element(begin, mid, begin + static_cast<std::ptrdiff_t>(n));
    double m = *mid;
    if (n % 2 == 0) {
        auto mid2 = begin + static_cast<std::ptrdiff_t>(n / 2 - 1);
        std::nth_element(begin, mid2, begin + static_cast<std::ptrdiff_t>(n));
        m = 0.5 * (m + *mid2);
    }
    return m;
}

double WaveletDenoiser::mad_sigma_from_detail(const std::array<double, 256>& coeffs,
                                              std::size_t start, std::size_t len) {
    // sigma ≈ MAD / 0.6745
    // MAD = median(|d - median(d)|)
    std::array<double, 256> tmp{};
    if (len == 0) return 0.0;

    // Copy detail band into tmp[0:len)
    for (std::size_t i = 0; i < len; ++i) tmp[i] = coeffs[start + i];
    double med = median(tmp, len);

    for (std::size_t i = 0; i < len; ++i) tmp[i] = std::fabs(tmp[i] - med);
    double mad = median(tmp, len);

    return mad / 0.6745;
}

void WaveletDenoiser::soft_threshold_range(std::array<double, 256>& coeffs,
                                           std::size_t start, std::size_t len,
                                           double T) {
    // Soft threshold: sign(x)*max(|x|-T,0)
    const std::size_t end = start + len;
    for (std::size_t i = start; i < end; ++i) {
        double x = coeffs[i];
        double ax = std::fabs(x);
        if (ax <= T) {
            coeffs[i] = 0.0;
        } else {
            coeffs[i] = (x > 0 ? 1.0 : -1.0) * (ax - T);
        }
    }
}
