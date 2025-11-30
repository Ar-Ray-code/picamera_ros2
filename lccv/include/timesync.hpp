#pragma once
#include <cstdint>
#include <deque>
#include <vector>
#include <algorithm>
#include <limits>
#include <cmath>

// Metadata keys written into CompletedRequest::post_process_metadata
inline constexpr const char *kMetaExposureHostSteadyNs = "ExposureHostSteadyNs";
inline constexpr const char *kMetaTimeSyncReady        = "TimeSyncReady";
inline constexpr const char *kMetaOffsetSensorToHostNs = "OffsetSensorToHostNs";
inline constexpr const char *kMetaTimesyncMadNs        = "TimesyncMadNs";

// Host steady "now" (CLOCK_MONOTONIC)
int64_t now_steady_ns();

struct TimeSyncConfig {
    double alpha = 0.025;          // EWMA smoothing for offset b
    int    warmup_frames = 15;     // frames before readiness check
    double ready_threshold_ns = 2.0e6; // ~2 ms MAD
    int    window = 60;            // residuals window for MAD
};

// Maps camera SensorTimestamp S (ns, camera/ISP clock) to host steady H (ns)
// using midpoint (Cristian/NTP) update: Hmid ~ (Hq + Hc)/2
class TimeSyncEstimator {
public:
    TimeSyncEstimator() = default;

    void configure(const TimeSyncConfig &cfg);
    void reset();

    // Update with one frame’s times: Hq = host queue time, Hc = host complete time, S = SensorTimestamp
    void update(int64_t Hq_ns, int64_t Hc_ns, int64_t S_ns);

    // Convert a SensorTimestamp S (ns) to host steady time (ns)
    int64_t sensor_to_host_steady(int64_t S_ns) const;

    // Diagnostics
    bool   ready() const;
    double offset_ns() const;       // b (host_steady - sensor)
    double median_abs_dev() const;  // MAD of residuals (ns)

private:
    // Affine mapping H ≈ a*S + b ; we only track b (a≈1)
    double a_ = 1.0;
    double b_ = 0.0;

    // Config
    TimeSyncConfig cfg_{};

    // State
    std::deque<double> residuals_ns_;
    bool ready_ = false;
    int  frames_seen_ = 0;
};
