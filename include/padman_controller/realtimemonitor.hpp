#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <vector>
#include <cmath>

using namespace std::chrono;

class RealtimeMonitor {
public:
    RealtimeMonitor(double expected_frequency)
        : expected_period_(1.0 / expected_frequency),
          last_time_(steady_clock::now()) {}

    void update() {
        auto now = steady_clock::now();
        double dt = duration<double>(now - last_time_).count();
        last_time_ = now;

        periods_.push_back(dt);
        if (periods_.size() > max_samples_) {
            periods_.erase(periods_.begin());  // Remove oldest entry
        }
    }

    void print_stats() {
        if (periods_.empty()) return;

        double sum = 0.0;
        double min_dt = std::numeric_limits<double>::max();
        double max_dt = std::numeric_limits<double>::lowest();

        for (double dt : periods_) {
            sum += dt;
            if (dt < min_dt) min_dt = dt;
            if (dt > max_dt) max_dt = dt;
        }

        double mean_dt = sum / (double)periods_.size();
        double mean_freq = 1.0 / mean_dt;

        // Compute jitter (standard deviation of period deviations)
        double jitter_sum = 0.0;
        for (double dt : periods_) {
            jitter_sum += (dt - mean_dt) * (dt - mean_dt);
        }
        double jitter = std::sqrt(jitter_sum / (double)periods_.size());

        RCLCPP_INFO(rclcpp::get_logger("RealtimeMonitor"),
                    "Freq: %.2f Hz (target: %.2f Hz), Jitter: %.3f ms, Min: %.3f ms, Max: %.3f ms",
                    mean_freq, 1.0 / expected_period_,
                    jitter * 1000.0, min_dt * 1000.0, max_dt * 1000.0);
    }

private:
    double expected_period_;
    std::vector<double> periods_;
    const size_t max_samples_ = 1000;
    time_point<steady_clock> last_time_;
};