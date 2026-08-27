/*
==============================================================================
SensorForge - Recording policy (data minimisation)
Part of the SensorForge AV HIL validation platform.

A recording-boundary policy that fits the bridge's actual data model. This is
DATA MINIMISATION, not privacy redaction, and the distinction is deliberate:

  - The bridge captures ROS messages as opaque serialized bytes through
    GenericSubscription. It has a topic name and a type NAME, and no field
    layout at all. Field-level redaction would require reintroducing typed
    deserialization for every message type, discarding the type-agnostic design
    that is the bridge's main architectural virtue.
  - By the time the WAL sees a payload it may also be compressed.

So the policy operates at the only granularity the data model actually exposes:
the STREAM. Three controls, all explicit in config:

  allowlist        if non-empty, ONLY these streams may be recorded.
  denylist         these streams are never recorded (checked after allowlist).
  metadata_only    these streams are recorded with a ZERO-LENGTH payload:
                   timestamp, sensor type and sequence are kept so rate, gap
                   and timing analysis still work, while the payload bytes
                   never reach the WAL or any offload destination.

Matching is exact stream name, plus an optional trailing '*' prefix wildcard.
No perception, no detection, no content inspection.
==============================================================================
*/

#pragma once

#include <string>
#include <vector>

namespace sensorforge::pipeline {

enum class PolicyDecision {
  kRecordFull,     // payload recorded verbatim
  kMetadataOnly,   // record header, drop payload bytes
  kDeny,           // nothing reaches the WAL or offload
};

inline const char * to_string(PolicyDecision d)
{
  switch (d) {
    case PolicyDecision::kRecordFull: return "record_full";
    case PolicyDecision::kMetadataOnly: return "metadata_only";
    case PolicyDecision::kDeny: return "deny";
  }
  return "unknown";
}

struct RecordingPolicy
{
  std::vector<std::string> allowlist;      // empty = allow all
  std::vector<std::string> denylist;
  std::vector<std::string> metadata_only;

  static bool matches(const std::string & pattern, const std::string & name)
  {
    if (!pattern.empty() && pattern.back() == '*') {
      const std::string prefix = pattern.substr(0, pattern.size() - 1);
      return name.size() >= prefix.size() &&
             name.compare(0, prefix.size(), prefix) == 0;
    }
    return pattern == name;
  }

  static bool in(const std::vector<std::string> & list, const std::string & name)
  {
    for (const auto & p : list) {
      if (matches(p, name)) {
        return true;
      }
    }
    return false;
  }

  /// Decide what may be recorded for @p stream. Deny wins over allow.
  PolicyDecision decide(const std::string & stream) const
  {
    if (in(denylist, stream)) {
      return PolicyDecision::kDeny;
    }
    if (!allowlist.empty() && !in(allowlist, stream)) {
      return PolicyDecision::kDeny;
    }
    if (in(metadata_only, stream)) {
      return PolicyDecision::kMetadataOnly;
    }
    return PolicyDecision::kRecordFull;
  }

  bool active() const
  {
    return !allowlist.empty() || !denylist.empty() || !metadata_only.empty();
  }
};

}  // namespace sensorforge::pipeline
