#pragma once
#include "Vector2D.hpp"
#include "AABBCollider.hpp"
#include <memory>
#include <deque>
#include <string>

enum class BodyType { Static, Dynamic };

class RigidBody {
public:
    RigidBody(std::string name, BodyType type, float mass, const Vector2D& initialPos, std::unique_ptr<Collider> collider)
        : m_name(std::move(name)), m_type(type), m_mass(mass), m_position(initialPos), m_collider(std::move(collider)) {
        m_invMass = (type == BodyType::Dynamic && mass > 0.0f) ? (1.0f / mass) : 0.0f;
        if (m_collider) {
            m_collider->UpdatePosition(m_position);
        }
    }

    void ApplyForce(const Vector2D& force) {
        if (m_type == BodyType::Dynamic) {
            m_forceAccumulator += force;
        }
    }

    void Integrate(float dt) {
        if (m_type == BodyType::Static) return;

        // Semi-implicit Euler
        Vector2D acceleration = m_forceAccumulator * m_invMass;
        m_velocity += acceleration * dt;
        m_position += m_velocity * dt;

        m_forceAccumulator = {0.0f, 0.0f};

        if (m_collider) {
            m_collider->UpdatePosition(m_position);
        }

        // Yörünge izi için geçmiş konum kaydı
        m_trajectoryHistory.push_back(m_position);
        if (m_trajectoryHistory.size() > 200) {
            m_trajectoryHistory.pop_front();
        }
    }

    // Getters & Setters
    const std::string& GetName() const { return m_name; }
    const Vector2D& GetPosition() const { return m_position; }
    void SetPosition(const Vector2D& pos) { 
        m_position = pos; 
        if (m_collider) m_collider->UpdatePosition(m_position);
    }

    const Vector2D& GetVelocity() const { return m_velocity; }
    void SetVelocity(const Vector2D& vel) { m_velocity = vel; }

    float GetInvMass() const { return m_invMass; }
    float GetRestitution() const { return m_restitution; }
    void SetRestitution(float e) { m_restitution = e; }

    Collider* GetCollider() const { return m_collider.get(); }
    BodyType GetType() const { return m_type; }
    const std::deque<Vector2D>& GetTrajectoryHistory() const { return m_trajectoryHistory; }

private:
    std::string m_name;
    BodyType m_type{BodyType::Dynamic};
    float m_mass{1.0f};
    float m_invMass{1.0f};
    float m_restitution{0.75f};

    Vector2D m_position{0.0f, 0.0f};
    Vector2D m_velocity{0.0f, 0.0f};
    Vector2D m_forceAccumulator{0.0f, 0.0f};

    std::unique_ptr<Collider> m_collider;
    std::deque<Vector2D> m_trajectoryHistory;
};