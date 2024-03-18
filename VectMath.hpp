#pragma once

#include <cmath>

template <typename T>
inline bool between(T val, T bound1, T bound2)
{
  if (bound1 > bound2) return val >= bound2 && val <= bound1;
  return val >= bound1 && val <= bound2;
}

template <typename T>
struct Vect2
{
  T x;
  T y;

  template <typename U>
  Vect2(const Vect2<U>& other)
  {
    x = (T)other.x;
    y = (T)other.y;
  }

  template <typename U>
  Vect2(U x, U y)
  {
    x = (T)x;
    y = (T)y;
  }

  inline Vect2<T> operator-(const Vect2<T>& rhs) const
  {
    return {x-rhs.x, y-rhs.y};
  }

  inline Vect2<T> operator+(const Vect2<T>& rhs) const
  {
    return {x+rhs.x, y+rhs.y};
  }

  inline Vect2<T> operator*(T rhs) const
  {
    return {x*rhs, y*rhs};
  }

  inline Vect2<T> operator*(const Vect2<T>& rhs) const
  {
    return {x*rhs.x, y*rhs.y};
  }

  inline Vect2<T> operator/(const Vect2<T>& rhs) const
  {
    return {x/rhs.x, y/rhs.y};
  }

  inline Vect2<T> transpose() const
  {
    return {y, x};
  }

  inline T getElementCloserToZero() const
  {
    return std::abs(x) < std::abs(y) ? x : y
  }

  inline T getElementFartherFromZero() const
  {
    return std::abs(x) > std::abs(y) ? x : y
  }

  template <typename OutT = T>
  inline OutT norm() const
  {
    return std::sqrt(std::pow((OutT)x, (OutT)2) + std::pow((OutT)y, (OutT)2));
  }

  template <typename OutT = T>
  inline OutT dist(const Vect2<T>& v) const
  {
    return std::sqrt(std::pow((OutT)(x-v.x), (OutT)2) + std::pow((OutT)(y-v.y), (OutT)2));
  }

  bool inside(const Vect2<T>& v) const
  {
    return between(x, 0, v.x) && between(y, (T)0, v.y);
  }
};

typedef Vect2<float> Vect2f;
typedef Vect2<double> Vect2d;
typedef Vect2<int64_t> Vect2i;