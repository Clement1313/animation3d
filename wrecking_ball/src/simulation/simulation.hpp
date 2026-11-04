#pragma once

#include <vector>

#include "cgp/cgp.hpp"

// from tp02
struct particle_structure
{
    cgp::vec3 p = { 0, 0, 0 }; // Position
    cgp::vec3 v = { 0, 0, 0 }; // Speed

    cgp::vec3 c = { 1, 1, 1 }; // Color
    float r = 0.1f; // Radius
    float m = 1.f; // mass
    bool active = false;
};

struct simulation_parameters
{
    float dt = 0.005f;
    int iterations = 4;
    float gravity = 9.81f;
    float damping = 0.1f;
    float restitution = 0.4f;
    float friction = 0.05f;
    float ball_mass = 50.0f;
    float brick_mass = 1.0f;
};

void simulate(particle_structure& ball, std::vector<particle_structure>& bricks,
              simulation_parameters const& sim, float dt);
