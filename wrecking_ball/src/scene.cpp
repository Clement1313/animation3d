#include "scene.hpp"

using namespace cgp;

namespace
{
    int const rows = 4;
    int const cols = 5;
    float const spacing = 0.52f;
    float const brick_size = 0.5f;
    vec3 const brick_color = { 0.65f, 0.25f, 0.15f };

    vec3 brick_position(int row, int col)
    {
        float const offset = (row % 2) * spacing / 2.0f;
        return { 0.0f, (col - (cols - 1) / 2.0f) * spacing + offset,
                 -0.75f + row * spacing };
    }
} // namespace

void scene_structure::initialize()
{
    // Camera
    camera_control.initialize(inputs, window);
    camera_control.set_rotation_axis_z();
    camera_control.look_at({ 3.0f, 2.0f, 2.0f }, { 0, 0, 0 }, { 0, 0, 1 });

    global_frame.initialize_data_on_gpu(mesh_primitive_frame());

    // Ground
    ground.initialize_data_on_gpu(mesh_primitive_quadrangle(
        { -5, -5, -1 }, { 5, -5, -1 }, { 5, 5, -1 }, { -5, 5, -1 }));

    // Wrecking ball
    sphere.initialize_data_on_gpu(mesh_primitive_sphere(0.45f));
    sphere.material.color = { 0.2f, 0.2f, 0.25f };

    // Cable (mesh rebuilt every frame in update_cable )
    cylinder.initialize_data_on_gpu(
        mesh_primitive_cylinder(0.035f, { 0, 0, 0 }, { 0, 0, -1.0f }));
    cylinder.material.color = { 0.4f, 0.4f, 0.4f };

    // Brick drawables
    mesh const brick_mesh = mesh_primitive_cube({ 0, 0, 0 }, brick_size);
    for (int k = 0; k < rows * cols; ++k)
    {
        mesh_drawable brick;
        brick.initialize_data_on_gpu(brick_mesh);
        brick.material.color = brick_color;
        bricks.push_back(brick);
    }
    reset_simulation();
}

void scene_structure::reset_simulation()
{
    // Wrecking ball
    timer.t = 0.0f;
    ball = particle_structure();
    ball.r = 0.45f;
    ball.m = sim.ball_mass;
    ball.c = { 0.2f, 0.2f, 0.25f };
    ball.active = true;
    update_ball();
    update_cable();

    bricks_state.clear();
    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            particle_structure b;
            b.p = brick_position(row, col);
            b.r = brick_size / 2.0f;
            b.m = sim.brick_mass;
            b.c = brick_color;
            b.active = false;
            bricks_state.push_back(b);
        }
    }

    // Rigid bricks
    rigid_bricks.clear();
    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            brick_structure b;
            b.initialize(brick_position(row, col), brick_size, 3);
            rigid_bricks.push_back(b);
        }
    }

    sync_drawables();
}

void scene_structure::sync_drawables()
{
    sphere.model.translation = ball.p;

    for (size_t k = 0; k < bricks.size(); ++k)
    {
        if (sim.rigid_bricks)
        {
            bricks[k].model.translation = rigid_bricks[k].com;
            bricks[k].model.rotation = rotation_transform::from_matrix(rigid_bricks[k].R);
        }
        else
        {
            bricks[k].model.translation = bricks_state[k].p;
            bricks[k].model.rotation = rotation_transform();
        }
    }
}

void scene_structure::display_frame()
{
    environment.camera_view = camera_control.camera_model.matrix_view();
    environment.light = camera_control.camera_model.position();

    if (gui.display_frame)
        draw(global_frame, environment);

    timer.update();

    float const dt = sim.dt * timer.scale;
    for (int k = 0; k < sim.iterations; ++k)
    {
        // analytic pendulum
        if (!sim.simulated_ball)
            update_ball();
        if (sim.rigid_bricks)
            simulate_rigid(ball, rigid_bricks, sim, pendulum, dt);
        else
            simulate(ball, bricks_state, sim, pendulum, dt);
    }

    if (!sim.simulated_ball)
        update_ball();
    update_cable();

    sync_drawables();

    draw(ground, environment);
    draw(cylinder, environment);
    draw(sphere, environment);

    for (mesh_drawable const& brick : bricks)
        draw(brick, environment);

    if (gui.display_wireframe)
    {
        draw_wireframe(sphere, environment);
        for (mesh_drawable const& brick : bricks)
            draw_wireframe(brick, environment);
    }
}

void scene_structure::display_gui()
{
    ImGui::Checkbox("Frame", &gui.display_frame);
    ImGui::SliderFloat("Time Scale", &timer.scale, 0.0f, 2.0f, "%.1f");
    ImGui::Checkbox("Wireframe", &gui.display_wireframe);

    ImGui::Separator();
    ImGui::Text("Simulation");
    ImGui::SliderFloat("dt", &sim.dt, 0.001f, 0.02f, "%.4f");
    ImGui::SliderInt("Iterations / frame", &sim.iterations, 1, 20);
    ImGui::SliderFloat("Gravity", &sim.gravity, 0.0f, 20.0f, "%.2f");
    ImGui::SliderFloat("Damping", &sim.damping, 0.0f, 2.0f, "%.2f");
    ImGui::SliderFloat("Restitution", &sim.restitution, 0.0f, 1.0f, "%.2f");
    ImGui::SliderFloat("Friction", &sim.friction, 0.0f, 1.0f, "%.2f");
    if (ImGui::Checkbox("Rigid bricks (shape matching)", &sim.rigid_bricks))
        reset_simulation();
    ImGui::SliderInt("Collision steps", &sim.collision_steps, 1, 20);

    bool changed = false;
    changed |=
        ImGui::SliderFloat("Ball mass", &sim.ball_mass, 1.0f, 200.0f, "%.1f");
    changed |=
        ImGui::SliderFloat("Brick mass", &sim.brick_mass, 0.1f, 10.0f, "%.2f");
    if (changed)
    {
        ball.m = sim.ball_mass;
        for (particle_structure& b : bricks_state)
            b.m = sim.brick_mass;
    }

    ImGui::Separator();
    ImGui::Text("Pendulum");
    if (ImGui::Checkbox("Simulated ball", &sim.simulated_ball))
        reset_simulation();
    if (ImGui::SliderFloat("Cable length", &pendulum.length, 2.0f, 4.0f,
                           "%.2f"))
        reset_simulation();
    if (ImGui::SliderAngle("Start angle", &pendulum.theta_max, 5.0f, 75.0f))
        reset_simulation();

    if (ImGui::Button("Reset"))
        reset_simulation();
}

void scene_structure::mouse_move_event()
{
    if (!inputs.keyboard.shift)
        camera_control.action_mouse_move();
}
void scene_structure::mouse_click_event()
{
    camera_control.action_mouse_click();
}
void scene_structure::keyboard_event()
{
    camera_control.action_keyboard();
}
void scene_structure::idle_frame()
{
    camera_control.idle_frame();
}

void scene_structure::update_ball()
{
    ball.p = pendulum.evaluate_position(timer.t, sim.gravity);
    ball.v = pendulum.evaluate_velocity(timer.t, sim.gravity);
}

void scene_structure::update_cable()
{
    cylinder.clear();
    cylinder.initialize_data_on_gpu(
        mesh_primitive_cylinder(0.035f, pendulum.pivot, ball.p));
    cylinder.material.color = { 0.4f, 0.4f, 0.4f };
}