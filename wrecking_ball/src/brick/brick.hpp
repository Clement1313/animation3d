#pragma once

#include "cgp/cgp.hpp"

struct brick_structure
{
    cgp::vec3 com = { 0, 0, 0 }; 
    cgp::vec3 com_reference = { 0, 0, 0 };
    cgp::mat3 R = cgp::mat3::build_identity(); 

    cgp::numarray<cgp::vec3> position;
    cgp::numarray<cgp::vec3> position_predict;
    cgp::numarray<cgp::vec3> position_reference;
    cgp::numarray<cgp::vec3> velocity;

    float particle_radius = 0.1f;
    bool active = false;

    void initialize(cgp::vec3 const& center, float size, int n);
    int size() const;
};
