// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================

#ifndef CH_SPH_UTILS_LOGGING_H
#define CH_SPH_UTILS_LOGGING_H

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <vector>
#include <unordered_map>

namespace chrono {
namespace fsi {
namespace sph {

// Define this macro to enable problem size logging
#define FSI_COUNT_LOGGING_ENABLED

#ifdef FSI_COUNT_LOGGING_ENABLED

class CountLogger {
  public:
    struct SizeStats {
        int64_t min = 0;
        int64_t max = 0;
        double avg = 0.0;
        int64_t total = 0;
        size_t call_count = 0;
    };

    static CountLogger& GetInstance() {
        static CountLogger instance;
        return instance;
    }

    // Add a count for a named event (e.g., number of candidate tets)
    void AddCount(const std::string& name, int64_t count) {
        if (!enabled_)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        counts_[name].push_back(count);
    }

    // Increment a count for a named event (e.g., faces inserted)
    void Increment(const std::string& name, int64_t delta = 1) {
        if (!enabled_)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        int64_t new_count = delta;
        if (!counts_[name].empty()) {
            new_count += counts_[name].back();
        }
        counts_[name].push_back(new_count);
    }

    // Set a specific value for a named event (useful for time steps)
    void SetValue(const std::string& name, int64_t value) {
        if (!enabled_)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        counts_[name].push_back(value);
    }

    SizeStats GetStats(const std::string& name) const {
        SizeStats stats;
        auto it = counts_.find(name);
        if (it == counts_.end())
            return stats;
        const auto& vals = it->second;
        if (vals.empty())
            return stats;
        stats.call_count = vals.size();
        stats.min = *std::min_element(vals.begin(), vals.end());
        stats.max = *std::max_element(vals.begin(), vals.end());
        stats.total = std::accumulate(vals.begin(), vals.end(), int64_t(0));
        // Always use call count from the HydroelasticQuery if available
        size_t call_count = stats.call_count;
        auto HydroI = counts_.find("HydroelasticQuery");
        if (HydroI != counts_.end()) {
            const auto& HydroTimes = HydroI->second;
            call_count = HydroTimes.size();
        }
        stats.avg = static_cast<double>(stats.total) / call_count;
        return stats;
    }

    void PrintStats() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << "\n=== Problem Size Statistics ===" << std::endl;
        for (const auto& [name, _] : counts_) {
            auto stats = GetStats(name);
            std::cout << "Problem Size: " << name << std::endl;
            std::cout << "  Calls: " << stats.call_count << std::endl;
            std::cout << "  Min: " << stats.min << std::endl;
            std::cout << "  Max: " << stats.max << std::endl;
            std::cout << "  Avg: " << stats.avg << std::endl;
            std::cout << "  Total: " << stats.total << std::endl;
            std::cout << std::endl;
        }
    }

    // Print all problem size statistics in JSON format
    void PrintStatsJson(const std::string& path = "") const { PrintStatsJson(path, ""); }

    // Overload: Print all problem size statistics in JSON format, with extra JSON
    // fields
    void PrintStatsJson(const std::string& path, const std::string& extra_json) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::ostringstream oss;
        oss << "{\n";
        oss << "  \"problem_sizes\": {\n";
        bool first = true;
        for (const auto& [name, _] : counts_) {
            if (!first)
                oss << ",\n";
            first = false;
            auto stats = GetStats(name);
            oss << "    \"" << name << "\": {"
                << "\"calls\": " << stats.call_count << ", "
                << "\"min\": " << stats.min << ", "
                << "\"max\": " << stats.max << ", "
                << "\"avg\": " << stats.avg << ", "
                << "\"total\": " << stats.total << "}";
        }
        oss << "\n  }";
        if (!extra_json.empty()) {
            oss << ",\n" << extra_json << "\n";
        } else {
            oss << "\n";
        }
        oss << "}";

        if (!path.empty()) {
            std::ofstream ofs(path);
            if (ofs.is_open()) {
                ofs << oss.str();
                ofs.close();
            } else {
                std::cerr << "Failed to open file for writing: " << path << std::endl;
            }
        } else {
            std::cout << oss.str() << std::endl;
        }
    }

    void Clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        counts_.clear();
    }

    void SetEnabled(bool enabled) { enabled_ = enabled; }
    bool IsEnabled() const { return enabled_; }

  private:
    CountLogger() = default;
    mutable std::mutex mutex_;
    std::map<std::string, std::vector<int64_t>> counts_;
    bool enabled_ = true;
};

// General-purpose quantity logger for tracking floating-point values (like time steps)
class QuantityLogger {
  public:
    struct QuantityStats {
        double min = 0.0;
        double max = 0.0;
        double avg = 0.0;
        double total = 0.0;
        size_t call_count = 0;
        double last_value = 0.0;
    };

    static QuantityLogger& GetInstance() {
        static QuantityLogger instance;
        return instance;
    }

    // Add a value for a named quantity (e.g., time step)
    void AddValue(const std::string& name, double value) {
        if (!enabled_)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        quantities_[name].push_back(value);
    }

    // Increment a quantity by a delta (useful for cumulative tracking)
    void Increment(const std::string& name, double delta = 1.0) {
        if (!enabled_)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        double new_value = delta;
        if (!quantities_[name].empty()) {
            new_value += quantities_[name].back();
        }
        quantities_[name].push_back(new_value);
    }

    // Set a specific value (overwrites the last value if it exists)
    void SetValue(const std::string& name, double value) {
        if (!enabled_)
            return;
        std::lock_guard<std::mutex> lock(mutex_);
        quantities_[name].push_back(value);
    }

    QuantityStats GetStats(const std::string& name) const {
        QuantityStats stats;
        auto it = quantities_.find(name);
        if (it == quantities_.end())
            return stats;
        const auto& vals = it->second;
        if (vals.empty())
            return stats;

        stats.call_count = vals.size();
        stats.min = *std::min_element(vals.begin(), vals.end());
        stats.max = *std::max_element(vals.begin(), vals.end());
        stats.total = std::accumulate(vals.begin(), vals.end(), 0.0);
        stats.avg = stats.total / stats.call_count;
        stats.last_value = vals.back();
        return stats;
    }

    void PrintStats() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << "\n=== Quantity Statistics ===" << std::endl;
        for (const auto& [name, _] : quantities_) {
            auto stats = GetStats(name);
            std::cout << "Quantity: " << name << std::endl;
            std::cout << "  Calls: " << stats.call_count << std::endl;
            std::cout << "  Min: " << stats.min << std::endl;
            std::cout << "  Max: " << stats.max << std::endl;
            std::cout << "  Avg: " << stats.avg << std::endl;
            std::cout << "  Total: " << stats.total << std::endl;
            std::cout << "  Last: " << stats.last_value << std::endl;
            std::cout << std::endl;
        }
    }

    // Print all quantity statistics in JSON format
    void PrintStatsJson(const std::string& path = "") const { PrintStatsJson(path, ""); }

    // Overload: Print all quantity statistics in JSON format, with extra JSON fields
    void PrintStatsJson(const std::string& path, const std::string& extra_json) const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::ostringstream oss;
        oss << "{\n";
        oss << "  \"quantities\": {\n";
        bool first = true;
        for (const auto& [name, _] : quantities_) {
            if (!first)
                oss << ",\n";
            first = false;
            auto stats = GetStats(name);
            oss << "    \"" << name << "\": {"
                << "\"calls\": " << stats.call_count << ", "
                << "\"min\": " << stats.min << ", "
                << "\"max\": " << stats.max << ", "
                << "\"avg\": " << stats.avg << ", "
                << "\"total\": " << stats.total << ", "
                << "\"last\": " << stats.last_value << "}";
        }
        oss << "\n  }";
        if (!extra_json.empty()) {
            oss << ",\n" << extra_json << "\n";
        } else {
            oss << "\n";
        }
        oss << "}";

        if (!path.empty()) {
            std::ofstream ofs(path);
            if (ofs.is_open()) {
                ofs << oss.str();
                ofs.close();
            } else {
                std::cerr << "Failed to open file for writing: " << path << std::endl;
            }
        } else {
            std::cout << oss.str() << std::endl;
        }
    }

    void Clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        quantities_.clear();
    }

    void SetEnabled(bool enabled) { enabled_ = enabled; }
    bool IsEnabled() const { return enabled_; }

    // Write all values for supplied quantities to a text file
    void WriteQuantityValuesToFile(const std::string& file_path, const std::vector<std::string>& quantity_names) const {
        std::lock_guard<std::mutex> lock(mutex_);

        // Try to create directory if it doesn't exist
        size_t last_slash = file_path.find_last_of('/');
        if (last_slash != std::string::npos) {
            std::string dir_path = file_path.substr(0, last_slash);
            std::filesystem::create_directories(dir_path);
        }

        // Try to create/open the file
        std::ofstream file(file_path);
        if (!file.is_open()) {
            std::cerr << "Failed to create/open file: " << file_path << std::endl;
            return;
        }

        // Check if quantity exists
        // Get their sizes
        size_t max_size = 0;
        std::unordered_map<std::string, size_t> sizes;
        for (const auto& qty : quantity_names) {
            auto it = quantities_.find(qty);
            if (it == quantities_.end()) {
                std::cerr << "Quantity '" << qty << "' not found" << std::endl;
                return;
            }
            sizes[qty] = it->second.size();
            max_size = std::max(max_size, sizes[qty]);
        }

        // Write header
        file << "index";
        for (const auto& qty : quantity_names) {
            file << "," << qty;
        }
        file << std::endl;

        // Write values in index,value1,value2,value3,... format
        for (size_t i = 0; i < max_size; ++i) {
            file << i;
            for (const auto& qty : quantity_names) {
                file << ",";
                auto it = quantities_.find(qty);
                if (it != quantities_.end() && i < it->second.size()) {
                    file << it->second[i];
                }
            }
            file << std::endl;
        }

        file.close();
    }

  private:
    QuantityLogger() = default;
    mutable std::mutex mutex_;
    std::map<std::string, std::vector<double>> quantities_;
    bool enabled_ = true;
};

// Convenience function to print all statistics from both loggers
inline void PrintAllStats() {
    CountLogger::GetInstance().PrintStats();
    QuantityLogger::GetInstance().PrintStats();
}

#else  // FSI_COUNT_LOGGING_ENABLED

class CountLogger {
  public:
    struct SizeStats {
        int64_t min = 0;
        int64_t max = 0;
        double avg = 0.0;
        int64_t total = 0;
        size_t call_count = 0;
    };
    static CountLogger& GetInstance() {
        static CountLogger instance;
        return instance;
    }
    void AddCount(const std::string&, int64_t) {}
    void Increment(const std::string&, int64_t = 1) {}
    void SetValue(const std::string&, int64_t) {}
    SizeStats GetStats(const std::string&) const { return SizeStats{}; }
    void PrintStats() const {}
    void Clear() {}
    void SetEnabled(bool) {}
    bool IsEnabled() const { return false; }
};

class QuantityLogger {
  public:
    struct QuantityStats {
        double min = 0.0;
        double max = 0.0;
        double avg = 0.0;
        double total = 0.0;
        size_t call_count = 0;
        double last_value = 0.0;
    };
    static QuantityLogger& GetInstance() {
        static QuantityLogger instance;
        return instance;
    }
    void AddValue(const std::string&, double) {}
    void Increment(const std::string&, double = 1.0) {}
    void SetValue(const std::string&, double) {}
    QuantityStats GetStats(const std::string&) const { return QuantityStats{}; }
    void PrintStats() const {}
    void Clear() {}
    void SetEnabled(bool) {}
    bool IsEnabled() const { return false; }
    void WriteQuantityValuesToFile(const std::string&, const std::vector<std::string>&) const {}
};

inline void PrintAllStats() {}
inline void PrintAllStatsJson(const std::string& = "", const std::string& = "") {}
inline void PrintTimeStepStats(const std::string& = "") {}

#endif  // FSI_COUNT_LOGGING_ENABLED

}  // namespace sph
}  // namespace fsi
}  // namespace chrono

#endif
