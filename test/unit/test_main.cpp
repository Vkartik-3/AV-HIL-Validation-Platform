/*
==============================================================================
SensorForge - GoogleTest assertion reporter (Extension G)

Self-registers (no custom main) so the suite works both standalone and under
ament_add_gtest, which supplies gtest_main. A listener counts assertion
FAILURES (gtest reports only failures to listeners) and prints the exact
executed-assertion TOTAL from the SF_* wrappers' counter at program end:

    [ASSERTIONS] total=82097 passed=82097 failed=0

so CI can assert the suite executes >= 5,000 assertions.
==============================================================================
*/

#include <cstdio>
#include <gtest/gtest.h>

#include "test_support.hpp"

namespace {

class AssertionReporter : public ::testing::EmptyTestEventListener
{
public:
  void OnTestPartResult(const ::testing::TestPartResult & r) override
  {
    if (r.failed()) {
      ++failed_;
    }
  }
  void OnTestProgramEnd(const ::testing::UnitTest &) override
  {
    const long long total = sftest::g_assertions.load();
    std::printf(
      "[ASSERTIONS] total=%lld passed=%lld failed=%lld\n",
      total, total - failed_, failed_);
  }

private:
  long long failed_ = 0;
};

// Register the listener at load time so no custom main() is required.
const bool g_registered = []() {
    ::testing::UnitTest::GetInstance()->listeners().Append(new AssertionReporter);
    return true;
  }();

}  // namespace
