/*
==============================================================================
SensorForge - JSON report serializer (Extension L)
==============================================================================
*/

#pragma once

#include <string>

#include "report/report_types.hpp"

namespace sensorforge::report {

/// Serialize a report to a pretty-printed JSON string.
std::string to_json(const ScenarioReport & r);

/// Write the JSON report to @p path. Returns false on I/O failure.
bool write_json(const ScenarioReport & r, const std::string & path);

}  // namespace sensorforge::report
