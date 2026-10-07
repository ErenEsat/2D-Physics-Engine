#pragma once
#include "CelestialBody.hpp"
#include <vector>
#include <memory>

class OrbitalWorld {
public:
    explicit OrbitalWorld(float G = 1000.0f) : m_G(G) {}

    void AddBody(CelestialBody body) {
        m_bodies.push_back(std::move(body));
    }

    void Step(float dt) {
        CalculateGravitationalForces();
        for (auto& body : m_bodies) {
            body.Integrate(dt);
        }
    }

    std::vector<CelestialBody>& GetBodies() { return m_bodies; }
    const std::vector<CelestialBody>& GetBodies() const { return m_bodies; }

private:
    void CalculateGravitationalForces() {
        size_t count = m_bodies.size();
        constexpr float softening = 25.0f; // Yakın geçişlerde fırlamayı önleyen sabitleme

        for (size_t i = 0; i < count; ++i) {
            for (size_t j = i + 1; j < count; ++j) {
                auto& b1 = m_bodies[i];
                auto& b2 = m_bodies[j];

                Vector2D diff = b2.position - b1.position;
                float distSq = diff.MagnitudeSquared() + softening;
                float dist = std::sqrt(distSq);

                // F = G * (m1 * m2) / r^2
                float forceMagnitude = (m_G * b1.mass * b2.mass) / distSq;
                Vector2D forceDir = diff * (1.0f / dist);
                Vector2D force = forceDir * forceMagnitude;

                // Etki - Tepki
                b1.ApplyForce(force);
                b2.ApplyForce(force * -1.0f);
            }
        }
    }

    float m_G; // Evrensel çekim sabiti
    std::vector<CelestialBody> m_bodies;
};