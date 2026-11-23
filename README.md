# Projet Ani3D — Boule de démolition

Une boule de démolition suspendue à un câble frappe un mur de briques et le fait s'effondrer (EPITA Ani3D 2026-2027).
Le code utilise CGP et ImGui ; Eigen (dans `wrecking_ball/third_party/`) sert uniquement à la décomposition polaire.

## Compiler et lancer

Il faut un compilateur C++17, OpenGL 3.3 et GLFW (`libglfw3-dev` sous Ubuntu).

```bash
cd wrecking_ball
make -j$(nproc)    # produit ./project
./project          # à lancer depuis wrecking_ball/ pour trouver shaders/
```

`make run` compile puis lance. Avec CMake :

```bash
cd wrecking_ball
cmake -S . -B build && cmake --build build -j$(nproc)
./project          # (CMake place aussi l'exécutable dans wrecking_ball/)
```

## Utilisation

- **Caméra** : clic gauche + glisser pour tourner, clic droit + glisser pour zoomer, Ctrl + clic gauche pour déplacer.
- **Launch** : démarre la simulation (au départ tout est en pause). **Pause** / **Resume** la fige ou la reprend. **Reset** remet la scène à zéro.
- **Simulated ball** : boule simulée (freinée par les briques) ou pendule analytique (masse infinie).
- **Rigid bricks (shape matching)** : briques rigides qui tournent, ou briques-sphères.
- **Ball mass**, **Brick mass**, **Restitution**, **Friction**, **Damping**, **Gravity**, **dt**, **Iterations / frame**, **Collision steps** : paramètres de la simulation.
- **Cable length**, **Start angle** : géométrie du pendule (remet la scène à zéro).
- **Wireframe**, **Frame** : affichage.

Changer certaines options (mode boule, mode briques, câble, angle) remet la simulation en pause : recliquer sur **Launch**.

## Organisation

- `wrecking_ball/src/scene.*` : scène et interface.
- `wrecking_ball/src/pendulum/` : pendule analytique.
- `wrecking_ball/src/simulation/` : briques-sphères (`simulation.cpp`) et shape matching (`simulation_rigid.cpp`).
- `wrecking_ball/src/brick/` : brique rigide (grille de particules).
- `rapport/` : rapport en LaTeX (`make` pour générer `rapport.pdf`).
