#include <SFML/Graphics.hpp>
#include "PhysicsWorld.hpp"
#include "AABBCollider.hpp"
#include <iostream>

enum class SceneMode {
    CollisionProjectiles,
    Pythagorean345Orbit
};

void LoadCollisionScene(PhysicsWorld& world) {
    world.Clear();
    world.SetUniformGravity(Vector2D{0.0f, 400.0f});
    world.SetNBodyGravity(false);

    // Zemin (Geniş ve statik platform)
    auto groundCol = std::make_unique<AABBCollider>(Vector2D{550.0f, 20.0f});
    auto ground = std::make_unique<RigidBody>("Ground", BodyType::Static, 0.0f, Vector2D{600.0f, 740.0f}, std::move(groundCol));
    world.AddBody(std::move(ground));

    // Dikey Düşen Kutu
    auto boxCol1 = std::make_unique<AABBCollider>(Vector2D{22.0f, 22.0f});
    auto box1 = std::make_unique<RigidBody>("Box_Drop", BodyType::Dynamic, 2.0f, Vector2D{400.0f, 150.0f}, std::move(boxCol1));
    box1->SetRestitution(0.75f);
    world.AddBody(std::move(box1));

    // Eğik Atılan Kutu
    auto boxCol2 = std::make_unique<AABBCollider>(Vector2D{18.0f, 18.0f});
    auto box2 = std::make_unique<RigidBody>("Box_Projectile", BodyType::Dynamic, 1.5f, Vector2D{120.0f, 480.0f}, std::move(boxCol2));
    box2->SetVelocity(Vector2D{160.0f, -280.0f}); // Parabolik fırlatma
    box2->SetRestitution(0.70f);
    world.AddBody(std::move(box2));
}

void LoadPythagoreanOrbitScene(PhysicsWorld& world, float centerX, float centerY) {
    world.Clear();
    world.SetUniformGravity(Vector2D{0.0f, 0.0f});
    world.SetNBodyGravity(true, 450000.0f); // Gerçekçi çekim sabiti

    float scale = 60.0f;

    // Kütle 3 (Kırmızı): Tepe Noktası (0, 3) - Başlangıç hızı 0
    Vector2D posA{centerX, centerY - (1.5f * scale)};
    auto colA = std::make_unique<AABBCollider>(Vector2D{10.0f, 10.0f});
    auto bodyA = std::make_unique<RigidBody>("Body_3", BodyType::Dynamic, 3.0f, posA, std::move(colA));
    world.AddBody(std::move(bodyA));

    // Kütle 4 (Yeşil): Dik Köşe (0, 0) - Başlangıç hızı 0
    Vector2D posB{centerX - (2.0f * scale), centerY + (1.5f * scale)};
    auto colB = std::make_unique<AABBCollider>(Vector2D{12.0f, 12.0f});
    auto bodyB = std::make_unique<RigidBody>("Body_4", BodyType::Dynamic, 4.0f, posB, std::move(colB));
    world.AddBody(std::move(bodyB));

    // Kütle 5 (Mavi): Taban Köşesi (4, 0) - Başlangıç hızı 0
    Vector2D posC{centerX + (2.0f * scale), centerY + (1.5f * scale)};
    auto colC = std::make_unique<AABBCollider>(Vector2D{14.0f, 14.0f});
    auto bodyC = std::make_unique<RigidBody>("Body_5", BodyType::Dynamic, 5.0f, posC, std::move(colC));
    world.AddBody(std::move(bodyC));
}

int main() {
    constexpr unsigned int WINDOW_WIDTH = 1200;
    constexpr unsigned int WINDOW_HEIGHT = 800;

    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), 
                            "[1] Carpismalar | [2] 3-4-5 Ucgeni Orbit | [R] Sifirla", 
                            sf::Style::Titlebar | sf::Style::Close);
    window.setFramerateLimit(60);

    PhysicsWorld world;
    SceneMode currentMode = SceneMode::CollisionProjectiles;
    LoadCollisionScene(world);

    sf::Clock clock;

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }

            if (event.type == sf::Event::KeyPressed) {
                if (event.key.code == sf::Keyboard::Num1 || event.key.code == sf::Keyboard::Numpad1) {
                    currentMode = SceneMode::CollisionProjectiles;
                    LoadCollisionScene(world);
                } else if (event.key.code == sf::Keyboard::Num2 || event.key.code == sf::Keyboard::Numpad2) {
                    currentMode = SceneMode::Pythagorean345Orbit;
                    LoadPythagoreanOrbitScene(world, WINDOW_WIDTH * 0.5f, WINDOW_HEIGHT * 0.5f);
                } else if (event.key.code == sf::Keyboard::R) {
                    if (currentMode == SceneMode::CollisionProjectiles) {
                        LoadCollisionScene(world);
                    } else {
                        LoadPythagoreanOrbitScene(world, WINDOW_WIDTH * 0.5f, WINDOW_HEIGHT * 0.5f);
                    }
                }
            }
        }

        float dt = clock.restart().asSeconds();
        if (dt > 0.05f) dt = 0.05f;

        // Sub-stepping: Yörüngelerin hassasiyetini ve çarpışma kararlılığını artırır
        constexpr int SUB_STEPS = 6;
        float subDt = dt / static_cast<float>(SUB_STEPS);
        for (int i = 0; i < SUB_STEPS; ++i) {
            world.Step(subDt);
        }

        // --- ÇİZİM DÖNGÜSÜ ---
        window.clear(sf::Color(14, 16, 24));

        for (const auto& body : world.GetBodies()) {
            // Yörünge İzi
            const auto& history = body->GetTrajectoryHistory();
            if (history.size() > 1) {
                sf::VertexArray trail(sf::LineStrip, history.size());
                for (size_t i = 0; i < history.size(); ++i) {
                    trail[i].position = sf::Vector2f(history[i].x, history[i].y);
                    float alpha = (static_cast<float>(i) / history.size()) * 230.0f;

                    if (body->GetName() == "Body_3") {
                        trail[i].color = sf::Color(255, 80, 80, static_cast<sf::Uint8>(alpha));
                    } else if (body->GetName() == "Body_4") {
                        trail[i].color = sf::Color(80, 240, 120, static_cast<sf::Uint8>(alpha));
                    } else if (body->GetName() == "Body_5") {
                        trail[i].color = sf::Color(80, 170, 255, static_cast<sf::Uint8>(alpha));
                    } else {
                        trail[i].color = sf::Color(100, 200, 255, static_cast<sf::Uint8>(alpha));
                    }
                }
                window.draw(trail);
            }

            // Kutu / Cisim Çizimi
            auto* aabb = dynamic_cast<AABBCollider*>(body->GetCollider());
            if (aabb) {
                Vector2D extents = aabb->GetHalfExtents();
                sf::RectangleShape rect(sf::Vector2f(extents.x * 2.0f, extents.y * 2.0f));
                rect.setOrigin(extents.x, extents.y);
                rect.setPosition(body->GetPosition().x, body->GetPosition().y);

                if (body->GetType() == BodyType::Static) {
                    rect.setFillColor(sf::Color(65, 70, 85));
                } else if (body->GetName() == "Body_3") {
                    rect.setFillColor(sf::Color(255, 80, 80));
                } else if (body->GetName() == "Body_4") {
                    rect.setFillColor(sf::Color(80, 240, 120));
                } else if (body->GetName() == "Body_5") {
                    rect.setFillColor(sf::Color(80, 170, 255));
                } else if (body->GetName() == "Box_Drop") {
                    rect.setFillColor(sf::Color(255, 150, 50));
                } else {
                    rect.setFillColor(sf::Color(80, 220, 140));
                }

                window.draw(rect);
            }
        }

        window.display();
    }

    return 0;
}