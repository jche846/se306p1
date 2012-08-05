#pragma once

#include <cmath>

namespace se306p1 {
  class Vector2 {
  public:
    double x_;
    double y_;

    inline Vector2(double x = 0.0, double y = 0.0) : x_(x), y_(y) { }

    inline Vector2(const Vector2 &from) : x_(from.x_), y_(from.y_) { }

    ~Vector2() {}

    inline bool operator==(const Vector2 &vector) const
    {
      return x_ == vector.x_ && x_ == vector.x_;
    }

    inline bool operator!=(const Vector2 &vector) const
    {
      return x_ != vector.x_ || x_ != vector.x_;
    }

    inline Vector2 operator+(const Vector2 &rhs) const
    {
      return Vector2(this->x_ + rhs.x_, this->y_ + rhs.y_);
    }

    inline Vector2 operator-(const Vector2 &rhs) const
    {
      return Vector2(this->x_ - rhs.x_, this->y_ - rhs.y_);
    }

    template<typename T>
    inline Vector2 operator*(T rhs) const
    {
      return Vector2(this->x_ * rhs, this->y_ * rhs);
    }

    template<typename T>
    inline Vector2 operator/(T rhs) const
    {
      return Vector2(this->x_ / rhs, this->y_ / rhs);
    }

    inline Vector2 operator*(const Vector2 &rhs) const
    {
      return Vector2(this->x_ * rhs.x_, this->y_ * rhs.y_);
    }

    inline Vector2 operator/(const Vector2 &rhs) const
    {
      return Vector2(this->x_ / rhs.x_, this->y_ / rhs.y_);
    }

    template<typename T>
    inline friend Vector2 operator*(T lhs, const Vector2 &rhs)
    {
      return Vector2(lhs * rhs.x_, lhs * rhs.y_);
    }

    template<typename T>
    inline friend Vector2 operator/(T lhs, const Vector2 &rhs)
    {
      return Vector2(lhs / rhs.x_, lhs / rhs.y_);
    }

    inline double Length() {
      return std::sqrt(this->LengthSquared());
    }

    inline double LengthSquared() {
      return this->x_ * this->x_ + this->y_ * this->y_;
    }

    inline Vector2 Normalized() {
      double len = this->Length();
      return Vector2(this->x_ / len, this->y_ / len);
    }
  };
}
