#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace px4_offboard {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kEps = 1e-12;

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  double &at(std::size_t index) {
    switch (index) {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
      default:
        throw std::out_of_range("Vec3 index");
    }
  }

  double at(std::size_t index) const {
    switch (index) {
      case 0:
        return x;
      case 1:
        return y;
      case 2:
        return z;
      default:
        throw std::out_of_range("Vec3 index");
    }
  }

  double &operator[](std::size_t index) { return at(index); }
  double operator[](std::size_t index) const { return at(index); }

  Vec3 operator+() const { return *this; }
  Vec3 operator-() const { return {-x, -y, -z}; }
  Vec3 &operator+=(const Vec3 &other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }
  Vec3 &operator-=(const Vec3 &other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }
  Vec3 &operator*=(double scale) {
    x *= scale;
    y *= scale;
    z *= scale;
    return *this;
  }
  Vec3 &operator/=(double scale) {
    x /= scale;
    y /= scale;
    z /= scale;
    return *this;
  }

  double squaredNorm() const { return x * x + y * y + z * z; }
  double norm() const { return std::sqrt(squaredNorm()); }
  bool finite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
};

inline Vec3 operator+(Vec3 left, const Vec3 &right) {
  left += right;
  return left;
}

inline Vec3 operator-(Vec3 left, const Vec3 &right) {
  left -= right;
  return left;
}

inline Vec3 operator*(Vec3 value, double scale) {
  value *= scale;
  return value;
}

inline Vec3 operator*(double scale, Vec3 value) {
  value *= scale;
  return value;
}

inline Vec3 operator/(Vec3 value, double scale) {
  value /= scale;
  return value;
}

inline double dot(const Vec3 &left, const Vec3 &right) {
  return left.x * right.x + left.y * right.y + left.z * right.z;
}

inline Vec3 cross(const Vec3 &left, const Vec3 &right) {
  return {left.y * right.z - left.z * right.y, left.z * right.x - left.x * right.z,
          left.x * right.y - left.y * right.x};
}

inline Vec3 hadamard(const Vec3 &left, const Vec3 &right) {
  return {left.x * right.x, left.y * right.y, left.z * right.z};
}

inline Vec3 normalized(const Vec3 &value) {
  const double norm = value.norm();
  if (!(norm > kEps) || !std::isfinite(norm)) {
    throw std::invalid_argument("cannot normalize zero/invalid vector");
  }
  return value / norm;
}

inline Vec3 clamped(const Vec3 &value, const Vec3 &limit) {
  return {std::clamp(value.x, -limit.x, limit.x), std::clamp(value.y, -limit.y, limit.y),
          std::clamp(value.z, -limit.z, limit.z)};
}

}  // namespace px4_offboard
