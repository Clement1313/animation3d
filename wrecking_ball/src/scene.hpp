#pragma once

#include <vector>

#include "cgp/cgp.hpp"
#include "environment.hpp"
#include "pendulum/pendulum.hpp"


using cgp::mesh_drawable;
using cgp::curve_drawable;
using cgp::mesh;
using cgp::numarray;
using cgp::vec3;

struct gui_parameters
{
    bool display_frame = true;
    bool display_wireframe = false;
};

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
    float ball_mass = 50.0f;
    float brick_mass = 1.0f;
};

// The structure of the custom scene
struct scene_structure : cgp::scene_inputs_generic
{
    // ****************************** //
    // Elements and shapes of the scene
    // ****************************** //
    camera_controller_orbit_euler camera_control;
    camera_projection_perspective camera_projection;
    window_structure window;

    mesh_drawable global_frame; // The standard global frame
    environment_structure environment; // Standard environment controler
    input_devices
        inputs; // Storage for inputs status (mouse, keyboard, window dimension)
    gui_parameters gui; // Standard GUI element storage
    // ****************************** //
    // Elements and shapes of the scene
    // ****************************** //

    cgp::timer_basic timer;

    mesh_drawable cube;
    mesh_drawable ground;
    mesh_drawable cylinder;
    mesh_drawable sphere;
    std::vector<mesh_drawable> bricks;

    simulation_parameters sim;
    particle_structure ball;
    std::vector<particle_structure> bricks_state;
	pendulum_structure pendulum;

    curve_drawable curve; // A set of vertices displayed as a curve
    // Functions
    // ****************************** //

    void initialize(); // Standard initialization to be called before the
                       // animation loop
    void
    display_frame(); // The frame display to be called within the animation loop
    void display_gui(); // The display of the GUI, also called within the
                        // animation loop

    void reset_simulation();
    void sync_drawables();

    void mouse_move_event();
    void mouse_click_event();
    void keyboard_event();
    void idle_frame();

	void update_ball();     // procedural motion of the wrecking ball
	void update_cable();    // orients the cable between pivot and ball
};