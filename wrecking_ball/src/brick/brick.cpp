#include "brick.hpp"

using namespace cgp;

void brick_structure::initialize(vec3 const& center, float size, int n)
{
    position.clear();
    velocity.clear();
    particle_radius = size / (2.0f * n); // the spheres of the grid touch each other

    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            for (int k = 0; k < n; ++k)
            {
                vec3 const offset = { -size / 2 + (i + 0.5f) * size / n, -size / 2 + (j + 0.5f) * size / n,
                                      -size / 2 + (k + 0.5f) * size / n };
                position.push_back(center + offset);
                velocity.push_back({ 0, 0, 0 });
            }
        }
    }

    position_predict = position;
    position_reference = position;
    com = average(position);
    com_reference = average(position_reference);
    R = mat3::build_identity();
    active = false;
}

int brick_structure::size() const
{
    return position.size();
}
