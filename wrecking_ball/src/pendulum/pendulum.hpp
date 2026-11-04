#pragma once

#include "cgp/cgp.hpp"

struct pendulum_structure
{
    cgp::vec3 pivot = { -1.5f, 0.0f, 3.5f }; // fixed attach point
    cgp::vec3 direction = { 1.0f, 0.0f,
                            0.0f }; // horizontal swing direction (unit vector)
    float length = 3.5f; // cable length
    float theta_max = 0.78f; // starting angle

    float evaluate_angle(float t, float g) const;
    float evaluate_angular_velocity(float t, float g) const;
    cgp::vec3 evaluate_position(float t, float g) const;
    cgp::vec3 evaluate_velocity(float t, float g) const;
};