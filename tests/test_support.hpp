#pragma once

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

inline void check(bool condition, const std::string &message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

inline void checkNear(double actual, double expected, double tolerance,
                      const std::string &message) {
  const bool valid = std::isfinite(actual) && std::isfinite(expected) &&
                     std::isfinite(tolerance) && tolerance >= 0.0;
  if (!valid || std::abs(actual - expected) > tolerance) {
    std::cerr << "FAIL: " << message << " expected=" << expected
              << " actual=" << actual << " tolerance=" << tolerance << '\n';
    std::exit(1);
  }
}
