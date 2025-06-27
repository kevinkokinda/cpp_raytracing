#pragma once
#include <cmath>
#include <iostream>

class Vec3 {
public:
    double x, y, z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double t) const { return Vec3(x * t, y * t, z * t); }
    Vec3 operator/(double t) const { return Vec3(x / t, y / t, z / t); }
    Vec3 operator*(const Vec3& v) const { return Vec3(x * v.x, y * v.y, z * v.z); }

    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3& operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vec3& operator*=(double t) { x *= t; y *= t; z *= t; return *this; }
    Vec3& operator/=(double t) { x /= t; y /= t; z /= t; return *this; }

    double length() const { return sqrt(x*x + y*y + z*z); }
    double lengthSquared() const { return x*x + y*y + z*z; }
    Vec3 normalize() const { return *this / length(); }
    Vec3 normalized() const { return normalize(); }
    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(const Vec3& v) const { 
        return Vec3(y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x); 
    }

    Vec3 reflect(const Vec3& normal) const {
        return *this - normal * 2 * this->dot(normal);
    }
    
    bool nearZero() const {
        const double eps = 1e-8;
        return (fabs(x) < eps) && (fabs(y) < eps) && (fabs(z) < eps);
    }

    friend std::ostream& operator<<(std::ostream& os, const Vec3& v) {
        return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    }
};

inline Vec3 operator*(double t, const Vec3& v) { return v * t; }
inline Vec3 operator-(const Vec3& v) { return Vec3(-v.x, -v.y, -v.z); }