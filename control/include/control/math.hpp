#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace control {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kEps = 1e-12;

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  double &operator[](std::size_t index) {
    return index == 0 ? x : (index == 1 ? y : z);
  }

  double operator[](std::size_t index) const {
    return index == 0 ? x : (index == 1 ? y : z);
  }

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

  Vec3 &operator*=(double scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }

  Vec3 &operator/=(double scalar) {
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
  }

  double squaredNorm() const { return x * x + y * y + z * z; }
  double norm() const { return std::sqrt(squaredNorm()); }

  bool finite() const {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }
};

inline Vec3 operator+(Vec3 lhs, const Vec3 &rhs) { return lhs += rhs; }
inline Vec3 operator-(Vec3 lhs, const Vec3 &rhs) { return lhs -= rhs; }
inline Vec3 operator*(Vec3 vector, double scalar) { return vector *= scalar; }
inline Vec3 operator*(double scalar, Vec3 vector) { return vector *= scalar; }
inline Vec3 operator/(Vec3 vector, double scalar) { return vector /= scalar; }

inline Vec3 hadamard(const Vec3 &lhs, const Vec3 &rhs) {
  return {lhs.x * rhs.x, lhs.y * rhs.y, lhs.z * rhs.z};
}

inline double dot(const Vec3 &lhs, const Vec3 &rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

inline Vec3 cross(const Vec3 &lhs, const Vec3 &rhs) {
  return {lhs.y * rhs.z - lhs.z * rhs.y,
          lhs.z * rhs.x - lhs.x * rhs.z,
          lhs.x * rhs.y - lhs.y * rhs.x};
}

inline Vec3 normalized(const Vec3 &vector) {
  const double norm_value = vector.norm();
  if (!(norm_value > kEps) || !std::isfinite(norm_value)) {
    throw std::invalid_argument("cannot normalize vector");
  }
  return vector / norm_value;
}

inline Vec3 clamped(const Vec3 &vector, const Vec3 &limit) {
  return {std::clamp(vector.x, -limit.x, limit.x),
          std::clamp(vector.y, -limit.y, limit.y),
          std::clamp(vector.z, -limit.z, limit.z)};
}

struct Mat3 {
  std::array<double, 9> a{1.0, 0.0, 0.0,
                          0.0, 1.0, 0.0,
                          0.0, 0.0, 1.0};

  static Mat3 identity() { return {}; }

  static Mat3 zero() {
    Mat3 matrix;
    matrix.a.fill(0.0);
    return matrix;
  }

  double &operator()(std::size_t row, std::size_t column) {
    return a[3 * row + column];
  }

  double operator()(std::size_t row, std::size_t column) const {
    return a[3 * row + column];
  }

  Vec3 column(std::size_t column_index) const {
    return {(*this)(0, column_index), (*this)(1, column_index), (*this)(2, column_index)};
  }

  void setColumn(std::size_t column_index, const Vec3 &vector) {
    (*this)(0, column_index) = vector.x;
    (*this)(1, column_index) = vector.y;
    (*this)(2, column_index) = vector.z;
  }

  Mat3 transpose() const {
    Mat3 result = zero();
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column_index = 0; column_index < 3; ++column_index) {
        result(row, column_index) = (*this)(column_index, row);
      }
    }
    return result;
  }

  bool finite() const {
    for (double value : a) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
    return true;
  }
};

inline Vec3 operator*(const Mat3 &matrix, const Vec3 &vector) {
  return {matrix(0, 0) * vector.x + matrix(0, 1) * vector.y + matrix(0, 2) * vector.z,
          matrix(1, 0) * vector.x + matrix(1, 1) * vector.y + matrix(1, 2) * vector.z,
          matrix(2, 0) * vector.x + matrix(2, 1) * vector.y + matrix(2, 2) * vector.z};
}

inline Mat3 operator*(const Mat3 &lhs, const Mat3 &rhs) {
  Mat3 result = Mat3::zero();
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column_index = 0; column_index < 3; ++column_index) {
      for (std::size_t k = 0; k < 3; ++k) {
        result(row, column_index) += lhs(row, k) * rhs(k, column_index);
      }
    }
  }
  return result;
}

inline Mat3 operator+(Mat3 lhs, const Mat3 &rhs) {
  for (std::size_t index = 0; index < lhs.a.size(); ++index) {
    lhs.a[index] += rhs.a[index];
  }
  return lhs;
}

inline Mat3 operator-(Mat3 lhs, const Mat3 &rhs) {
  for (std::size_t index = 0; index < lhs.a.size(); ++index) {
    lhs.a[index] -= rhs.a[index];
  }
  return lhs;
}

inline Mat3 operator*(Mat3 matrix, double scalar) {
  for (double &value : matrix.a) {
    value *= scalar;
  }
  return matrix;
}

inline Mat3 operator*(double scalar, Mat3 matrix) { return matrix * scalar; }

inline Mat3 diagonal(const Vec3 &diagonal_values) {
  Mat3 matrix = Mat3::zero();
  matrix(0, 0) = diagonal_values.x;
  matrix(1, 1) = diagonal_values.y;
  matrix(2, 2) = diagonal_values.z;
  return matrix;
}

inline Mat3 hat(const Vec3 &vector) {
  Mat3 matrix = Mat3::zero();
  matrix(0, 1) = -vector.z;
  matrix(0, 2) = vector.y;
  matrix(1, 0) = vector.z;
  matrix(1, 2) = -vector.x;
  matrix(2, 0) = -vector.y;
  matrix(2, 1) = vector.x;
  return matrix;
}

inline Vec3 vee(const Mat3 &matrix) {
  return {matrix(2, 1), matrix(0, 2), matrix(1, 0)};
}

inline Mat3 skew(const Mat3 &matrix) {
  return 0.5 * (matrix - matrix.transpose());
}

inline double determinant(const Mat3 &matrix) {
  return matrix(0, 0) *
             (matrix(1, 1) * matrix(2, 2) - matrix(1, 2) * matrix(2, 1)) -
         matrix(0, 1) *
             (matrix(1, 0) * matrix(2, 2) - matrix(1, 2) * matrix(2, 0)) +
         matrix(0, 2) *
             (matrix(1, 0) * matrix(2, 1) - matrix(1, 1) * matrix(2, 0));
}

inline Mat3 inverse(const Mat3 &matrix) {
  const double determinant_value = determinant(matrix);
  if (std::abs(determinant_value) < kEps) {
    throw std::invalid_argument("singular matrix");
  }

  Mat3 result = Mat3::zero();
  result(0, 0) = matrix(1, 1) * matrix(2, 2) - matrix(1, 2) * matrix(2, 1);
  result(0, 1) = matrix(0, 2) * matrix(2, 1) - matrix(0, 1) * matrix(2, 2);
  result(0, 2) = matrix(0, 1) * matrix(1, 2) - matrix(0, 2) * matrix(1, 1);
  result(1, 0) = matrix(1, 2) * matrix(2, 0) - matrix(1, 0) * matrix(2, 2);
  result(1, 1) = matrix(0, 0) * matrix(2, 2) - matrix(0, 2) * matrix(2, 0);
  result(1, 2) = matrix(0, 2) * matrix(1, 0) - matrix(0, 0) * matrix(1, 2);
  result(2, 0) = matrix(1, 0) * matrix(2, 1) - matrix(1, 1) * matrix(2, 0);
  result(2, 1) = matrix(0, 1) * matrix(2, 0) - matrix(0, 0) * matrix(2, 1);
  result(2, 2) = matrix(0, 0) * matrix(1, 1) - matrix(0, 1) * matrix(1, 0);
  return result * (1.0 / determinant_value);
}

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
    const double norm_value = std::sqrt(squaredNorm());
    if (!(norm_value > kEps) || !std::isfinite(norm_value)) {
      throw std::invalid_argument("invalid quaternion");
    }
    return {w / norm_value, x / norm_value, y / norm_value, z / norm_value};
  }

  Quat canonical() const {
    const Quat quaternion = normalized();
    return quaternion.w < 0.0
               ? Quat{-quaternion.w, -quaternion.x, -quaternion.y, -quaternion.z}
               : quaternion;
  }

  Quat inverse() const {
    const Quat quaternion = normalized();
    return {quaternion.w, -quaternion.x, -quaternion.y, -quaternion.z};
  }

  Quat operator*(const Quat &rhs) const {
    return {w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
            w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
            w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
            w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w};
  }

  Mat3 toMat3() const {
    const Quat quaternion = normalized();
    const double ww = quaternion.w * quaternion.w;
    const double xx = quaternion.x * quaternion.x;
    const double yy = quaternion.y * quaternion.y;
    const double zz = quaternion.z * quaternion.z;

    Mat3 matrix = Mat3::zero();
    matrix(0, 0) = ww + xx - yy - zz;
    matrix(0, 1) = 2.0 * (quaternion.x * quaternion.y - quaternion.w * quaternion.z);
    matrix(0, 2) = 2.0 * (quaternion.x * quaternion.z + quaternion.w * quaternion.y);
    matrix(1, 0) = 2.0 * (quaternion.x * quaternion.y + quaternion.w * quaternion.z);
    matrix(1, 1) = ww - xx + yy - zz;
    matrix(1, 2) = 2.0 * (quaternion.y * quaternion.z - quaternion.w * quaternion.x);
    matrix(2, 0) = 2.0 * (quaternion.x * quaternion.z - quaternion.w * quaternion.y);
    matrix(2, 1) = 2.0 * (quaternion.y * quaternion.z + quaternion.w * quaternion.x);
    matrix(2, 2) = ww - xx - yy + zz;
    return matrix;
  }

  Vec3 dcmZ() const { return toMat3().column(2); }

  static Quat fromAxisAngle(Vec3 axis, double angle_rad) {
    axis = control::normalized(axis);
    const double half_angle = 0.5 * angle_rad;
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

inline Mat3 rotationFromColumns(const Vec3 &column_0, const Vec3 &column_1,
                                const Vec3 &column_2) {
  Mat3 rotation = Mat3::zero();
  rotation.setColumn(0, column_0);
  rotation.setColumn(1, column_1);
  rotation.setColumn(2, column_2);
  return rotation;
}

}  // namespace control
