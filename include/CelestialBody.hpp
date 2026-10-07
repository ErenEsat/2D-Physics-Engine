#pragma once
#include "Vector2D.hpp"
#include <vector>
#include <deque>

struct CelestialBody {
    std::string name;
    float mass{1.0f};
    float radius{5.0f};
    bool isStatic{false};

    Vector2D position{0.0f, 0.0f};
    Vector2D velocity{0.0f, 0.0f};
    Vector2D forceAccumulator{0.0f, 0.0f};

    std::deque<Vector2D> trajectoryHistory;
    static constexpr size_t MAX_TRAIL_POINTS = 300;

    CelestialBody(std::string bodyName, float m, float r, Vector2D pos, Vector2D vel, bool fixed = false)
        : name(std::move(bodyName)), mass(m), radius(r), isStatic(fixed), position(pos), velocity(vel) {}

    void ApplyForce(const Vector2D& force) {
        if (!isStatic) {
            forceAccumulator += force;
        }
    }

    void Integrate(float dt) {
        if (isStatic) return;

        // a = F / m
        Vector2D acceleration = forceAccumulator * (1.0f / mass);
        
        // Semi-Implicit Euler
        velocity += acceleration * dt;
        position += velocity * dt;

        forceAccumulator = {0.0f, 0.0f};

        trajectoryHistory.push_back(position);
        if (trajectoryHistory.size() > MAX_TRAIL_POINTS) {
            trajectoryHistory.pop_front();
        }
    }
};