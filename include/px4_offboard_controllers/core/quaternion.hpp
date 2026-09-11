#pragma once

#include "px4_offboard_controllers/core/matrix3.hpp"

#include <cmath>
#include <stdexcept>

namespace px4_offboard {

struct Quat {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};

  bool finite() const {
    return std::isfinite(w) && std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }

  double squaredNorm() const { return w * w + x * x + y * y + z * z; }

  Quat normalized() const {
    const double norm = std::sqrt(squaredNorm());
    if (!(norm > kEps) || !std::isfinite(norm)) throw std::invalid_argument("invalid quaternion");
    return {w / norm, x / norm, y / norm, z / norm};
  }

  Quat canonical() const {
    const Quat value = normalized();
    return value.w < 0.0 ? Quat{-value.w, -value.x, -value.y, -value.z} : value;
  }

  Quat inverse() const {
    const Quat value = normalized();
    return {value.w, -value.x, -value.y, -value.z};
  }

  Quat operator*(const Quat &right) const {
    return {w * right.w - x * right.x - y * right.y - z * right.z,
            w * right.x + x * right.w + y * right.z - z * right.y,
            w * right.y - x * right.z + y * right.w + z * right.x,
            w * right.z + x * right.y - y * right.x + z * right.w};
  }

  Mat3 toMat3() const {
    const Quat q = normalized();
    const double ww = q.w * q.w;
    const double xx = q.x * q.x;
    const double yy = q.y * q.y;
    const double zz = q.z * q.z;
    Mat3 matrix = Mat3::zero();
    matrix(0, 0) = ww + xx - yy - zz;
    matrix(0, 1) = 2.0 * (q.x * q.y - q.w * q.z);
    matrix(0, 2) = 2.0 * (q.x * q.z + q.w * q.y);
    matrix(1, 0) = 2.0 * (q.x * q.y + q.w * q.z);
    matrix(1, 1) = ww - xx + yy - zz;
    matrix(1, 2) = 2.0 * (q.y * q.z - q.w * q.x);
    matrix(2, 0) = 2.0 * (q.x * q.z - q.w * q.y);
    matrix(2, 1) = 2.0 * (q.y * q.z + q.w * q.x);
    matrix(2, 2) = ww - xx - yy + zz;
    return matrix;
  }

  Vec3 dcmZ() const { return toMat3().column(2); }

  static Quat fromAxisAngle(Vec3 axis, double angle) {
    axis = px4_offboard::normalized(axis);
    const double half_angle = 0.5 * angle;
    const double sine = std::sin(half_angle);
    return {std::cos(half_angle), axis.x * sine, axis.y * sine, axis.z * sine};
  }

  static Quat fromMat3(const Mat3 &matrix) {
    const double trace = matrix(0, 0) + matrix(1, 1) + matrix(2, 2);
    Quat quaternion;
    if (trace > 0.0) {
      const double scale = 2.0 * std::sqrt(trace + 1.0);
      quaternion.w = 0.25 * scale;
      quaternion.x = (matrix(2, 1) - matrix(1, 2)) / scale;
      quaternion.y = (matrix(0, 2) - matrix(2, 0)) / scale;
      quaternion.z = (matrix(1, 0) - matrix(0, 1)) / scale;
    } else if (matrix(0, 0) > matrix(1, 1) && matrix(0, 0) > matrix(2, 2)) {
      const double scale =
          2.0 * std::sqrt(1.0 + matrix(0, 0) - matrix(1, 1) - matrix(2, 2));
      quaternion.w = (matrix(2, 1) - matrix(1, 2)) / scale;
      quaternion.x = 0.25 * scale;
      quaternion.y = (matrix(0, 1) + matrix(1, 0)) / scale;
      quaternion.z = (matrix(0, 2) + matrix(2, 0)) / scale;
    } else if (matrix(1, 1) > matrix(2, 2)) {
      const double scale =
          2.0 * std::sqrt(1.0 + matrix(1, 1) - matrix(0, 0) - matrix(2, 2));
      quaternion.w = (matrix(0, 2) - matrix(2, 0)) / scale;
      quaternion.x = (matrix(0, 1) + matrix(1, 0)) / scale;
      quaternion.y = 0.25 * scale;
      quaternion.z = (matrix(1, 2) + matrix(2, 1)) / scale;
    } else {
      const double scale =
          2.0 * std::sqrt(1.0 + matrix(2, 2) - matrix(0, 0) - matrix(1, 1));
      quaternion.w = (matrix(1, 0) - matrix(0, 1)) / scale;
      quaternion.x = (matrix(0, 2) + matrix(2, 0)) / scale;
      quaternion.y = (matrix(1, 2) + matrix(2, 1)) / scale;
      quaternion.z = 0.25 * scale;
    }
    return quaternion.normalized();
  }

  double rotationDistance(const Quat &other) const {
    const Quat lhs = normalized();
    const Quat rhs = other.normalized();
    double cosine_half_angle =
        std::abs(lhs.w * rhs.w + lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z);
    cosine_half_angle = std::clamp(cosine_half_angle, -1.0, 1.0);
    return 2.0 * std::acos(cosine_half_angle);
  }
};

}  // namespace px4_offboard
