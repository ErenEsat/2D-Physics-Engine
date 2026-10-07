#pragma once
#include "Vector2D.hpp"

enum class ColliderType { AABB };

class Collider {
public:
    explicit Collider(ColliderType type) : m_type(type) {}
    virtual ~Collider() = default;

    ColliderType GetType() const { return m_type; }
    virtual void UpdatePosition(const Vector2D& position) = 0;
    virtual bool CheckCollision(const Collider& other) const = 0;

private:
    ColliderType m_type;
};