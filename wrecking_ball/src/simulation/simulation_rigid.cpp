#include "simulation.hpp"
#include "../../third_party/eigen/Eigen/Core"
#include "../../third_party/eigen/Eigen/SVD"

using namespace cgp;


mat3 polar_decomposition(mat3 const& M)
{
    Eigen::Matrix3f A;
    A << M(0, 0), M(0, 1), M(0, 2), M(1, 0), M(1, 1), M(1, 2), M(2, 0), M(2, 1), M(2, 2);
    Eigen::JacobiSVD<Eigen::Matrix3f> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3f const R = svd.matrixU() * (svd.matrixV().transpose());

    return { R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2) };
}

// Collision between the particles of the active bricks and the ground
void collision_with_ground(std::vector<brick_structure>& bricks, simulation_parameters const& sim)
{
    float const ground_z = -1.0f;
    for (size_t kb = 0; kb < bricks.size(); ++kb)
    {
        brick_structure& b = bricks[kb];
        if (!b.active)
            continue;
        for (int k = 0; k < b.size(); ++k)
        {
            vec3& p = b.position_predict[k];
            if (p.z - b.particle_radius < ground_z)
            {
                p.z = ground_z + b.particle_radius;
                // friction with the ground: lose a part of the horizontal displacement
                p.x = b.position[k].x + (1 - sim.friction) * (p.x - b.position[k].x);
                p.y = b.position[k].y + (1 - sim.friction) * (p.y - b.position[k].y);
            }
        }
    }
}

// Collision between the ball and the particles of the bricks.
void collision_ball_bricks(particle_structure const& ball, vec3& ball_predict, float w_ball,
                           std::vector<brick_structure>& bricks, simulation_parameters const& sim)
{
    for (size_t kb = 0; kb < bricks.size(); ++kb)
    {
        brick_structure& b = bricks[kb];
        if (norm(b.com - ball_predict) > ball.r + 0.6f)
            continue;

        float const w_particle = b.size() / sim.brick_mass;
        float const min_distance = ball.r + b.particle_radius;
        for (int k = 0; k < b.size(); ++k)
        {
            vec3& p = b.position_predict[k];
            vec3 const delta = p - ball_predict;
            float const distance = norm(delta);
            if (distance < min_distance && distance > 1e-6f)
            {
                vec3 const n = delta / distance;
                float const overlap = min_distance - distance;
                p += (overlap * w_particle / (w_ball + w_particle)) * n;
                ball_predict -= (overlap * w_ball / (w_ball + w_particle)) * n;
                b.active = true; // wake up
            }
        }
    }
}

// Collision between the particles of two different bricks
void collision_between_bricks(std::vector<brick_structure>& bricks)
{
    size_t const N = bricks.size();

    std::vector<bounding_box> bbox;
    for (size_t kb = 0; kb < N; ++kb)
    {
        bounding_box box;
        box.initialize(bricks[kb].position_predict);
        box.extends(bricks[kb].particle_radius);
        bbox.push_back(box);
    }

    for (size_t i = 0; i < N; ++i)
    {
        for (size_t j = i + 1; j < N; ++j)
        {
            brick_structure& b1 = bricks[i];
            brick_structure& b2 = bricks[j];
            if (!b1.active && !b2.active)
                continue;
            if (!bounding_box::collide(bbox[i], bbox[j]))
                continue;

            float const min_distance = b1.particle_radius + b2.particle_radius;
            for (int p1 = 0; p1 < b1.size(); ++p1)
            {
                for (int p2 = 0; p2 < b2.size(); ++p2)
                {
                    vec3& a = b1.position_predict[p1];
                    vec3& b = b2.position_predict[p2];
                    vec3 const delta = b - a;
                    float const distance = norm(delta);
                    if (distance < min_distance && distance > 1e-6f)
                    {
                        vec3 const n = delta / distance;
                        float const overlap = min_distance - distance;
                        a -= 0.5f * overlap * n;
                        b += 0.5f * overlap * n;
                        b1.active = true;
                        b2.active = true;
                    }
                }
            }
        }
    }
}

// Shape matching
void shape_matching(std::vector<brick_structure>& bricks)
{
    for (size_t kb = 0; kb < bricks.size(); ++kb)
    {
        brick_structure& b = bricks[kb];
        if (!b.active)
            continue;

        b.com = average(b.position_predict);

        mat3 M = mat3::build_zero();
        for (int k = 0; k < b.size(); ++k)
        {
            vec3 const r = b.position_predict[k] - b.com;
            vec3 const r_ref = b.position_reference[k] - b.com_reference;
            M += tensor_product(r, r_ref);
        }
        b.R = polar_decomposition(M);

        // New predicted positions: p = R (p_ref - com_ref) + com
        for (int k = 0; k < b.size(); ++k)
            b.position_predict[k] = b.R * (b.position_reference[k] - b.com_reference) + b.com;
    }
}

void simulate_rigid(particle_structure& ball, std::vector<brick_structure>& bricks, simulation_parameters const& sim,
                    pendulum_structure const& pendulum, float dt)
{
    vec3 const g = { 0, 0, -sim.gravity };
    size_t const N = bricks.size();

    for (size_t kb = 0; kb < N; ++kb)
    {
        brick_structure& b = bricks[kb];
        for (int k = 0; k < b.size(); ++k)
        {
            if (b.active)
            {
                b.velocity[k] = b.velocity[k] * (1 - dt * sim.damping) + dt * g;
                b.position_predict[k] = b.position[k] + dt * b.velocity[k];
            }
            else
                b.position_predict[k] = b.position[k]; // asleep
        }
    }

    float w_ball = 0.0f;
    if (sim.simulated_ball)
    {
        w_ball = 1.0f / ball.m;
        ball.v = ball.v + dt * g;
    }
    vec3 ball_predict = ball.p + dt * ball.v;

    for (int k_step = 0; k_step < sim.collision_steps; ++k_step)
    {
        collision_with_ground(bricks, sim);
        collision_ball_bricks(ball, ball_predict, w_ball, bricks, sim);
        collision_between_bricks(bricks);
        shape_matching(bricks);

        if (sim.simulated_ball)
        {
            vec3 const n = normalize(ball_predict - pendulum.pivot);
            ball_predict = pendulum.pivot + pendulum.length * n;
        }
    }

    for (size_t kb = 0; kb < N; ++kb)
    {
        brick_structure& b = bricks[kb];
        if (!b.active)
            continue;
        for (int k = 0; k < b.size(); ++k)
        {
            b.velocity[k] = (b.position_predict[k] - b.position[k]) / dt;
            b.position[k] = b.position_predict[k];
        }
        b.com = average(b.position);
    }

    if (sim.simulated_ball)
    {
        ball.v = (ball_predict - ball.p) / dt;
        ball.p = ball_predict;

        vec3 const n = normalize(ball.p - pendulum.pivot);
        ball.v = ball.v - dot(ball.v, n) * n;
    }
}
