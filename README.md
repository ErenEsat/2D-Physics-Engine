# 2D Physics & Orbital Simulation Engine

A modular, real-time 2D physics and orbital mechanics simulation library developed in **Modern C++ (C++17)** with **SFML** graphics rendering. 

The engine showcases robust object-oriented system design, decoupling core numerical integrators and collision detection routines from presentation layers.

---

## Interactive Simulation Modes

Switch scenes dynamically via keyboard input at runtime:

- **[Key 1] Rigid-Body Collisions & Projectile Motion:**
  - Dynamic box drop and projectile parabolic arc.
  - Impulse-based momentum resolution satisfying Newton's coefficient of restitution.
  - **Positional Penetration Correction:** Eliminates entity overlap and floor sinking bugs.
- **[Key 2] Pythagorean Three-Body Orbital Problem:**
  - Simulates the classic astrophysical chaotic 3-body problem (masses 3, 4, and 5 placed on a 3-4-5 right triangle).
  - $N$-Body mutual gravitational attraction:
    $$\vec{F}_{ij} = G \frac{m_i m_j}{(r^2 + \epsilon^2)} \hat{r}_{ij}$$
  - Dynamic trajectory trail visualization displaying the evolution of chaotic motion.
- **[Key R]:** Reset the active simulation scene.

---

## Architectural Highlights

- **Polymorphic Abstraction:** Abstract `Collider` base class featuring pure virtual `CheckCollision` and `UpdatePosition` methods, decoupled from dynamic `RigidBody` objects.
- **Memory Safety & RAII:** Dynamic entities owned and managed exclusively via `std::unique_ptr` and `std::vector` containers; zero manual pointer manipulation or heap leaks.
- **Numerical Stability:** Semi-implicit Euler integration paired with multi-step sub-stepping (6 sub-steps per frame) to prevent orbital divergence during close approaches.

---

## Architecture Overview

```text
├── include/
│   ├── Vector2D.hpp       # 2D Vector math & operator overloads
│   ├── Collider.hpp       # Abstract polymorphic collider base interface
│   ├── AABBCollider.hpp   # Axis-Aligned Bounding Box implementation
│   ├── RigidBody.hpp      # Kinetics, restitution, mass inversion & trajectory
│   └── PhysicsWorld.hpp   # Euler stepping, N-body solver & collision resolution
└── src/
    └── main.cpp           # SFML rendering loop, scene orchestration & user input
```

---

## Build and Run

### Prerequisites
- C++17 compatible compiler (GCC / MinGW-w64)
- SFML 2.6+

### Compiling with g++ (MSYS2 UCRT64)
```bash
g++ -std=c++17 -Iinclude src/main.cpp -lsfml-graphics -lsfml-window -lsfml-system -o PhysicsEngine.exe
```

### Execution
```bash
.\PhysicsEngine.exe
```

---

## License
MIT License