#pragma once

#include <cmath>

namespace se306p1 {
class Vector2 {
 public:
  double x_; ///< x value.
  double y_; ///< y value.

  /**
   * Initialize a vector with an x and y value.
   */
  explicit inline Vector2(double x = 0.0, double y = 0.0)
      : x_(x),
        y_(y) {
  }

  /**
   * Copy constructor.
   */
  inline Vector2(const Vector2 &from)
      : x_(from.x_),
        y_(from.y_) {
  }

  ~Vector2() {
  }

  /**
   * Check the equality of two vectors.
   */
  inline bool operator==(const Vector2 &vector) const {
    return this->x_ == vector.x_ && this->x_ == vector.x_;
  }

  /**
   * Check the inequality of two vectors.
   */
  inline bool operator!=(const Vector2 &vector) const {
    return this->x_ != vector.x_ || this->x_ != vector.x_;
  }

  /**
   * Add two vectors.
   */
  inline Vector2 operator+(const Vector2 &rhs) const {
    return Vector2(this->x_ + rhs.x_, this->y_ + rhs.y_);
  }

  /**
   * Subtract two vectors.
   */
  inline Vector2 operator-(const Vector2 &rhs) const {
    return Vector2(this->x_ - rhs.x_, this->y_ - rhs.y_);
  }

  /**
   * Multiply a vector with a scalar.
   */
  template<typename T>
  inline Vector2 operator*(T rhs) const {
    return Vector2(this->x_ * rhs, this->y_ * rhs);
  }

  /**
   * Divide a vector by a scalar.
   */
  template<typename T>
  inline Vector2 operator/(T rhs) const {
    return Vector2(this->x_ / rhs, this->y_ / rhs);
  }

  /**
   * Multiply two vectors.
   */
  inline Vector2 operator*(const Vector2 &rhs) const {
    return Vector2(this->x_ * rhs.x_, this->y_ * rhs.y_);
  }

  /**
   * Divide two vectors.
   */
  inline Vector2 operator/(const Vector2 &rhs) const {
    return Vector2(this->x_ / rhs.x_, this->y_ / rhs.y_);
  }

  /**
   * Multiply a scalar with a vector.
   */
  template<typename T>
  inline friend Vector2 operator*(T lhs, const Vector2 &rhs) {
    return Vector2(lhs * rhs.x_, lhs * rhs.y_);
  }

  /**
   * Divide a scalar by a vector.
   */
  template<typename T>
  inline friend Vector2 operator/(T lhs, const Vector2 &rhs) {
    return Vector2(lhs / rhs.x_, lhs / rhs.y_);
  }

  /**
   * Compute the length of the vector.
   */
  inline double Length() {
    return std::sqrt(this->LengthSquared());
  }

  /**
   * Compute the length squared of the vector.
   */
  inline double LengthSquared() {
    return this->x_ * this->x_ + this->y_ * this->y_;
  }

  /**
   * Get a normalized vector.
   */
  inline Vector2 Normalized() {
    double len = this->Length();
    return Vector2(this->x_ / len, this->y_ / len);
  }
};
}
