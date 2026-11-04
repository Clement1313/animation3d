#include "pendulum.hpp"

#include <cmath>

using namespace cgp;

// theta < 0: the ball starts behind the pivot and swings toward +direction
float pendulum_structure::evaluate_angle(float t, float g) const
{
    float const omega = std::sqrt(g / length);
    return -theta_max * std::cos(omega * t);
}

float pendulum_structure::evaluate_angular_velocity(float t, float g) const
{
    float const omega = std::sqrt(g / length);
    return theta_max * omega * std::sin(omega * t);
}

vec3 pendulum_structure::evaluate_position(float t, float g) const
{
    float const theta = evaluate_angle(t, g);
    vec3 const down = { 0.0f, 0.0f, -1.0f };
    return pivot
        + length * (std::sin(theta) * direction + std::cos(theta) * down);
}

// Derivative of the position: L * theta' * (cos(theta) d + sin(theta) z)
vec3 pendulum_structure::evaluate_velocity(float t, float g) const
{
    float const theta = evaluate_angle(t, g);
    float const dtheta = evaluate_angular_velocity(t, g);
    vec3 const up = { 0.0f, 0.0f, 1.0f };
    return length * dtheta
        * (std::cos(theta) * direction + std::sin(theta) * up);
}