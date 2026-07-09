/*
==============================================================================
SensorForge - HTML report generator (Extension L)
==============================================================================
*/

#pragma once

#include <string>

#include "report/report_types.hpp"

namespace sensorforge::report {

/// Render a self-contained HTML summary (no external assets).
std::string to_html(const ScenarioReport & r);

bool write_html(const ScenarioReport & r, const std::string & path);

}  // namespace sensorforge::report
