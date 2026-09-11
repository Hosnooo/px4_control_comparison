#pragma once

#include "px4_offboard_controllers/core/vector3.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace px4_offboard {

struct Mat3 {
  std::array<double, 9> a{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

  static Mat3 identity() { return {}; }
  static Mat3 zero() {
    Mat3 matrix;
    matrix.a.fill(0.0);
    return matrix;
  }

  double &at(std::size_t row, std::size_t column) {
    if (row > 2 || column > 2) throw std::out_of_range("Mat3 index");
    return a[3 * row + column];
  }

  double at(std::size_t row, std::size_t column) const {
    if (row > 2 || column > 2) throw std::out_of_range("Mat3 index");
    return a[3 * row + column];
  }

  double &operator()(std::size_t row, std::size_t column) { return at(row, column); }
  double operator()(std::size_t row, std::size_t column) const { return at(row, column); }

  Vec3 column(std::size_t index) const { return {at(0, index), at(1, index), at(2, index)}; }

  void setColumn(std::size_t index, const Vec3 &value) {
    at(0, index) = value.x;
    at(1, index) = value.y;
    at(2, index) = value.z;
  }

  Mat3 transpose() const {
    Mat3 result = zero();
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        result(row, column) = at(column, row);
      }
    }
    return result;
  }

  bool finite() const {
    for (double value : a) {
      if (!std::isfinite(value)) return false;
    }
    return true;
  }
};

inline Vec3 operator*(const Mat3 &matrix, const Vec3 &vector) {
  return {matrix(0, 0) * vector.x + matrix(0, 1) * vector.y + matrix(0, 2) * vector.z,
          matrix(1, 0) * vector.x + matrix(1, 1) * vector.y + matrix(1, 2) * vector.z,
          matrix(2, 0) * vector.x + matrix(2, 1) * vector.y + matrix(2, 2) * vector.z};
}

inline Mat3 operator*(const Mat3 &left, const Mat3 &right) {
  Mat3 result = Mat3::zero();
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      for (std::size_t inner = 0; inner < 3; ++inner) {
        result(row, column) += left(row, inner) * right(inner, column);
      }
    }
  }
  return result;
}

inline Mat3 operator+(Mat3 left, const Mat3 &right) {
  for (std::size_t index = 0; index < left.a.size(); ++index) left.a[index] += right.a[index];
  return left;
}

inline Mat3 operator-(Mat3 left, const Mat3 &right) {
  for (std::size_t index = 0; index < left.a.size(); ++index) left.a[index] -= right.a[index];
  return left;
}

inline Mat3 operator*(Mat3 matrix, double scale) {
  for (double &value : matrix.a) value *= scale;
  return matrix;
}

inline Mat3 operator*(double scale, Mat3 matrix) { return matrix * scale; }

inline Mat3 diagonal(const Vec3 &diagonal_value) {
  Mat3 matrix = Mat3::zero();
  matrix(0, 0) = diagonal_value.x;
  matrix(1, 1) = diagonal_value.y;
  matrix(2, 2) = diagonal_value.z;
  return matrix;
}

inline Mat3 hat(const Vec3 &value) {
  Mat3 matrix = Mat3::zero();
  matrix(0, 1) = -value.z;
  matrix(0, 2) = value.y;
  matrix(1, 0) = value.z;
  matrix(1, 2) = -value.x;
  matrix(2, 0) = -value.y;
  matrix(2, 1) = value.x;
  return matrix;
}

inline Vec3 vee(const Mat3 &matrix) { return {matrix(2, 1), matrix(0, 2), matrix(1, 0)}; }

inline Mat3 skew(const Mat3 &matrix) { return 0.5 * (matrix - matrix.transpose()); }

inline double determinant(const Mat3 &matrix) {
  return matrix(0, 0) *
             (matrix(1, 1) * matrix(2, 2) - matrix(1, 2) * matrix(2, 1)) -
         matrix(0, 1) *
             (matrix(1, 0) * matrix(2, 2) - matrix(1, 2) * matrix(2, 0)) +
         matrix(0, 2) *
             (matrix(1, 0) * matrix(2, 1) - matrix(1, 1) * matrix(2, 0));
}

inline Mat3 rotationFromColumns(const Vec3 &first, const Vec3 &second, const Vec3 &third) {
  Mat3 matrix = Mat3::zero();
  matrix.setColumn(0, first);
  matrix.setColumn(1, second);
  matrix.setColumn(2, third);
  return matrix;
}

inline Mat3 inverse(const Mat3 &matrix) {
  const double determinant_value = determinant(matrix);
  if (std::abs(determinant_value) < kEps) throw std::invalid_argument("singular matrix");

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

}  // namespace px4_offboard
