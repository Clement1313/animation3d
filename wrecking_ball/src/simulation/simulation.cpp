#include "simulation.hpp"

using namespace cgp;

void simulate(particle_structure& ball, std::vector<particle_structure>& bricks,
              simulation_parameters const& sim,
              pendulum_structure const& pendulum, float dt)
{
    vec3 const g = { 0, 0, -sim.gravity };
    float const ground_z = -1.0f;
    size_t const N = bricks.size();

    // Gravity + damping
    for (size_t k = 0; k < N; ++k)
    {
        particle_structure& b = bricks[k];
        if (!b.active)
            continue;
        vec3 const f = b.m * g;
        b.v = (1 - sim.damping * dt) * b.v + dt * f / b.m;
    }

    // Simulated ball: gravity only
    if (sim.simulated_ball)
        ball.v = ball.v + dt * g;

    // Collision brick / ground
    for (size_t k = 0; k < N; ++k)
    {
        particle_structure& b = bricks[k];
        if (!b.active)
            continue;
        if (b.p.z - b.r < ground_z)
        {
            b.p.z = ground_z + b.r;
            if (b.v.z < 0.0f)
                b.v.z = -sim.restitution * b.v.z;
            b.v.x *= (1 - sim.friction);
            b.v.y *= (1 - sim.friction);
        }
    }

    // Collision ball / bricks
    float const w_ball = sim.simulated_ball ? 1.0f / ball.m : 0.0f;
    for (size_t k = 0; k < N; ++k)
    {
        particle_structure& b = bricks[k];
        vec3 const delta = b.p - ball.p;
        float const distance = norm(delta);
        float const min_distance = ball.r + b.r;

        if (distance < min_distance && distance > 1e-6f)
        {
            vec3 const n = delta / distance;
            float const w_brick = 1.0f / b.m;
            b.active = true; // wake up

            float const v_rel = dot(b.v - ball.v, n);
            if (v_rel < 0.0f)
            {
                float const impulse =
                    -(1.0f + sim.restitution) * v_rel / (w_ball + w_brick);
                b.v += (impulse * w_brick) * n;
                ball.v -= (impulse * w_ball) * n;
            }

            float const overlap = min_distance - distance;
            b.p += (overlap * w_brick / (w_ball + w_brick)) * n;
            ball.p -= (overlap * w_ball / (w_ball + w_brick)) * n;
        }
    }

    // Collision brick / brick
    for (size_t i = 0; i < N; ++i)
    {
        for (size_t j = i + 1; j < N; ++j)
        {
            particle_structure& p1 = bricks[i];
            particle_structure& p2 = bricks[j];
            if (!p1.active && !p2.active)
                continue;

            vec3 const delta = p2.p - p1.p;
            float const distance = norm(delta);
            float const min_distance = p1.r + p2.r;

            if (distance < min_distance && distance > 1e-6f)
            {
                vec3 const n = delta / distance;
                p1.active = true;
                p2.active = true;

                float const v_rel = dot(p2.v - p1.v, n);
                if (v_rel < 0.0f)
                {
                    float const impulse = -(1.0f + sim.restitution) * v_rel
                        / (1.0f / p1.m + 1.0f / p2.m);
                    p1.v -= (impulse / p1.m) * n;
                    p2.v += (impulse / p2.m) * n;
                }

                float const overlap = min_distance - distance;
                float const w1 = 1.0f / p1.m;
                float const w2 = 1.0f / p2.m;
                p1.p -= (overlap * w1 / (w1 + w2)) * n;
                p2.p += (overlap * w2 / (w1 + w2)) * n;
            }
        }
    }

    for (size_t k = 0; k < N; ++k)
    {
        if (bricks[k].active)
            bricks[k].p = bricks[k].p + dt * bricks[k].v;
    }

    if (sim.simulated_ball)
    {
        ball.p = ball.p + dt * ball.v;

        vec3 const n = normalize(ball.p - pendulum.pivot);
        ball.p = pendulum.pivot + pendulum.length * n;
        ball.v = ball.v - dot(ball.v, n) * n;
    }
}
