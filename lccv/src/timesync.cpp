#include "timesync.hpp"
#include <chrono>

int64_t now_steady_ns() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

void TimeSyncEstimator::configure(const TimeSyncConfig &cfg) { cfg_ = cfg; }

void TimeSyncEstimator::reset() {
    a_ = 1.0; b_ = 0.0; ready_ = false; frames_seen_ = 0; residuals_ns_.clear();
}

void TimeSyncEstimator::update(int64_t Hq_ns, int64_t Hc_ns, int64_t S_ns) {
    const double Hmid = 0.5 * (double(Hq_ns) + double(Hc_ns));
    const double off  = Hmid - double(S_ns);               // instantaneous offset
    b_ = (1.0 - cfg_.alpha) * b_ + cfg_.alpha * off;       // EWMA offset

    const double pred = a_ * double(S_ns) + b_;
    const double res  = Hmid - pred;
    residuals_ns_.push_back(std::abs(res));
    if ((int)residuals_ns_.size() > cfg_.window) residuals_ns_.pop_front();

    frames_seen_++;
    if (frames_seen_ >= cfg_.warmup_frames) {
        const double mad = median_abs_dev();
        ready_ = (mad <= cfg_.ready_threshold_ns);
    }
}

int64_t TimeSyncEstimator::sensor_to_host_steady(int64_t S_ns) const {
    return (int64_t)(a_ * double(S_ns) + b_);
}

bool TimeSyncEstimator::ready() const { return ready_; }
double TimeSyncEstimator::offset_ns() const { return b_; }

double TimeSyncEstimator::median_abs_dev() const {
    if (residuals_ns_.empty())
        return std::numeric_limits<double>::infinity();
    std::vector<double> v(residuals_ns_.begin(), residuals_ns_.end());
    std::nth_element(v.begin(), v.begin() + v.size()/2, v.end());
    const double med = v[v.size()/2];
    for (double &x : v) x = std::abs(x - med);
    std::nth_element(v.begin(), v.begin() + v.size()/2, v.end());
    return v[v.size()/2];
}
