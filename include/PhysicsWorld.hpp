#pragma once
#include "RigidBody.hpp"
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>

class PhysicsWorld {
public:
    explicit PhysicsWorld(const Vector2D& uniformGravity = {0.0f, 0.0f}) 
        : m_uniformGravity(uniformGravity) {}

    void Clear() {
        m_bodies.clear();
    }

    void SetUniformGravity(const Vector2D& g) { m_uniformGravity = g; }
    void SetNBodyGravity(bool enabled, float G = 350000.0f) { 
        m_enableNBody = enabled; 
        m_G = G; 
    }

    void AddBody(std::unique_ptr<RigidBody> body) {
        m_bodies.push_back(std::move(body));
    }

    void Step(float dt) {
        // 1. N-Body Çekimi (Aktifse)
        if (m_enableNBody) {
            CalculateNBodyGravitation();
        }

        // 2. Yerçekimi ve Entegrasyon
        for (auto& body : m_bodies) {
            if (body->GetType() == BodyType::Dynamic) {
                if (m_uniformGravity.MagnitudeSquared() > 0.0f) {
                    Vector2D gravityForce = m_uniformGravity * (1.0f / body->GetInvMass());
                    body->ApplyForce(gravityForce);
                }
            }
            body->Integrate(dt);
        }

        // 3. Çarpışmaları Çöz (Sadece yörünge modu kapalıyken)
        if (!m_enableNBody) {
            ResolveCollisions();
        }
    }

    const std::vector<std::unique_ptr<RigidBody>>& GetBodies() const { return m_bodies; }

private:
    void CalculateNBodyGravitation() {
        size_t count = m_bodies.size();
        constexpr float softening = 600.0f; // Yakın geçişlerde sonsuz ivmeyi önler

        for (size_t i = 0; i < count; ++i) {
            for (size_t j = i + 1; j < count; ++j) {
                auto& b1 = m_bodies[i];
                auto& b2 = m_bodies[j];

                Vector2D diff = b2->GetPosition() - b1->GetPosition();
                float distSq = diff.MagnitudeSquared() + softening;
                float dist = std::sqrt(distSq);

                float m1 = 1.0f / b1->GetInvMass();
                float m2 = 1.0f / b2->GetInvMass();

                // F = G * (m1 * m2) / r^2
                float forceMag = (m_G * m1 * m2) / distSq;
                Vector2D forceDir = diff * (1.0f / dist);
                Vector2D force = forceDir * forceMag;

                b1->ApplyForce(force);
                b2->ApplyForce(force * -1.0f);
            }
        }
    }

    void ResolveCollisions() {
        size_t count = m_bodies.size();
        for (size_t i = 0; i < count; ++i) {
            for (size_t j = i + 1; j < count; ++j) {
                auto& a = m_bodies[i];
                auto& b = m_bodies[j];

                if (!a->GetCollider() || !b->GetCollider()) continue;
                if (a->GetType() == BodyType::Static && b->GetType() == BodyType::Static) continue;

                auto* colA = dynamic_cast<AABBCollider*>(a->GetCollider());
                auto* colB = dynamic_cast<AABBCollider*>(b->GetCollider());
                if (!colA || !colB) continue;

                Vector2D diff = b->GetPosition() - a->GetPosition();
                Vector2D aHalf = colA->GetHalfExtents();
                Vector2D bHalf = colB->GetHalfExtents();

                // Örtüşme (Overlap) miktarları
                float overlapX = (aHalf.x + bHalf.x) - std::abs(diff.x);
                float overlapY = (aHalf.y + bHalf.y) - std::abs(diff.y);

                if (overlapX > 0.0f && overlapY > 0.0f) {
                    Vector2D normal;
                    float penetration = 0.0f;

                    // En küçük örtüşme ekseni çarpışma normalidir
                    if (overlapX < overlapY) {
                        penetration = overlapX;
                        normal = (diff.x > 0.0f) ? Vector2D{1.0f, 0.0f} : Vector2D{-1.0f, 0.0f};
                    } else {
                        penetration = overlapY;
                        normal = (diff.y > 0.0f) ? Vector2D{0.0f, 1.0f} : Vector2D{0.0f, -1.0f};
                    }

                    // A. KONUM DÜZELTMESİ (Cisimleri iç içe geçmekten anında kurtarır)
                    float totalInvMass = a->GetInvMass() + b->GetInvMass();
                    if (totalInvMass > 0.0f) {
                        Vector2D correction = normal * (penetration / totalInvMass);
                        if (a->GetType() == BodyType::Dynamic) a->SetPosition(a->GetPosition() - correction * a->GetInvMass());
                        if (b->GetType() == BodyType::Dynamic) b->SetPosition(b->GetPosition() + correction * b->GetInvMass());
                    }

                    // B. İMPULS HESABI (Esnek Sekme)
                    Vector2D rv = b->GetVelocity() - a->GetVelocity();
                    float velAlongNormal = rv.Dot(normal);

                    if (velAlongNormal < 0.0f) { // Birbirlerine doğru hareket ediyorlarsa
                        float e = std::min(a->GetRestitution(), b->GetRestitution());
                        float impulseMag = -(1.0f + e) * velAlongNormal / totalInvMass;
                        Vector2D impulse = normal * impulseMag;

                        if (a->GetType() == BodyType::Dynamic) a->SetVelocity(a->GetVelocity() - impulse * a->GetInvMass());
                        if (b->GetType() == BodyType::Dynamic) b->SetVelocity(b->GetVelocity() + impulse * b->GetInvMass());
                    }
                }
            }
        }
    }

    Vector2D m_uniformGravity{0.0f, 0.0f};
    bool m_enableNBody{false};
    float m_G{350000.0f};
    std::vector<std::unique_ptr<RigidBody>> m_bodies;
};