/*
==============================================================================
SensorForge - Prometheus metric registry (Extension M)
Part of the SensorForge AV HIL validation platform.

A small thread-safe registry of gauges and counters with labels that renders to
the Prometheus text exposition format. No external dependency. Non-ROS;
unit-testable standalone.
==============================================================================
*/

#pragma once

#include <cstdint>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace sensorforge::metrics {

using Labels = std::map<std::string, std::string>;

class Registry
{
public:
  /// Set a gauge (absolute value) for the given labelset.
  void set_gauge(const std::string & name, double value, const Labels & labels = {})
  {
    std::lock_guard<std::mutex> lk(mtx_);
    auto & fam = families_[name];
    fam.type = "gauge";
    fam.series[key(labels)] = {labels, value};
  }

  /// Convenience: gauge labelled by a single "sensor" dimension.
  void set_sensor_gauge(const std::string & name, const std::string & sensor, double value)
  {
    set_gauge(name, value, Labels{{"sensor", sensor}});
  }

  /// Add to a counter (monotonic) for the given labelset.
  void add_counter(const std::string & name, double delta = 1.0, const Labels & labels = {})
  {
    std::lock_guard<std::mutex> lk(mtx_);
    auto & fam = families_[name];
    fam.type = "counter";
    fam.series[key(labels)].value += delta;
    fam.series[key(labels)].labels = labels;
  }

  /// Set help text for a metric family.
  void set_help(const std::string & name, const std::string & help)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    families_[name].help = help;
  }

  /// Render the registry in Prometheus text exposition format.
  std::string render() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    std::ostringstream os;
    for (const auto & [name, fam] : families_) {
      if (!fam.help.empty()) {
        os << "# HELP " << name << " " << fam.help << "\n";
      }
      os << "# TYPE " << name << " " << (fam.type.empty() ? "gauge" : fam.type) << "\n";
      for (const auto & [k, s] : fam.series) {
        (void)k;
        os << name;
        if (!s.labels.empty()) {
          os << "{";
          bool first = true;
          for (const auto & [lk2, lv] : s.labels) {
            if (!first) {os << ",";}
            os << lk2 << "=\"" << escape(lv) << "\"";
            first = false;
          }
          os << "}";
        }
        os << " " << format(s.value) << "\n";
      }
    }
    return os.str();
  }

private:
  struct Series
  {
    Labels labels;
    double value = 0.0;
  };
  struct Family
  {
    std::string type;
    std::string help;
    std::map<std::string, Series> series;
  };

  static std::string key(const Labels & labels)
  {
    std::string k;
    for (const auto & [n, v] : labels) {
      k += n;
      k += '=';
      k += v;
      k += ';';
    }
    return k;
  }

  static std::string escape(const std::string & s)
  {
    std::string out;
    for (char c : s) {
      if (c == '"' || c == '\\') {out += '\\';}
      if (c == '\n') {out += "\\n"; continue;}
      out += c;
    }
    return out;
  }

  static std::string format(double v)
  {
    std::ostringstream os;
    os << v;
    return os.str();
  }

  mutable std::mutex mtx_;
  std::map<std::string, Family> families_;
};

}  // namespace sensorforge::metrics
