#pragma once
#include <cmath>

struct Vector2D {
    float x{0.0f};
    float y{0.0f};

    constexpr Vector2D() = default;
    constexpr Vector2D(float xCoord, float yCoord) : x(xCoord), y(yCoord) {}

    Vector2D operator+(const Vector2D& other) const { return {x + other.x, y + other.y}; }
    Vector2D operator-(const Vector2D& other) const { return {x - other.x, y - other.y}; }
    Vector2D operator*(float scalar) const { return {x * scalar, y * scalar}; }

    Vector2D& operator+=(const Vector2D& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    float Dot(const Vector2D& other) const { return (x * other.x) + (y * other.y); }
    float MagnitudeSquared() const { return (x * x) + (y * y); }
    float Magnitude() const { return std::sqrt(MagnitudeSquared()); }

    Vector2D Normalized() const {
        float mag = Magnitude();
        return (mag > 0.0f) ? (*this * (1.0f / mag)) : Vector2D{0.0f, 0.0f};
    }
};