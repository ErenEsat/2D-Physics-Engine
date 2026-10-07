#pragma once
#include "Collider.hpp"

class AABBCollider : public Collider {
public:
    AABBCollider(const Vector2D& halfExtents)
        : Collider(ColliderType::AABB), m_halfExtents(halfExtents) {}

    void UpdatePosition(const Vector2D& position) override {
        m_min = position - m_halfExtents;
        m_max = position + m_halfExtents;
    }

    bool CheckCollision(const Collider& other) const override {
        if (other.GetType() == ColliderType::AABB) {
            const auto& aabb = static_cast<const AABBCollider&>(other);
            return (m_min.x <= aabb.m_max.x && m_max.x >= aabb.m_min.x) &&
                   (m_min.y <= aabb.m_max.y && m_max.y >= aabb.m_min.y);
        }
        return false;
    }

    const Vector2D& GetMin() const { return m_min; }
    const Vector2D& GetMax() const { return m_max; }
    const Vector2D& GetHalfExtents() const { return m_halfExtents; }

private:
    Vector2D m_halfExtents;
    Vector2D m_min{0.0f, 0.0f};
    Vector2D m_max{0.0f, 0.0f};
};