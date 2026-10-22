#include "scene.hpp"

// Test scene to check correct compilation and run
//  The code should display a rotating cube and a textured quad on top of a
//  circular ground.

using namespace cgp;

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
    sphere.model.translation = { -1.5f, 0.0f, 1.25f };

    // Chain
    cylinder.initialize_data_on_gpu(
        mesh_primitive_cylinder(0.035f, { 0, 0, -0.4f }, { 0, 0, 0.4f }));
    cylinder.material.color = { 0.4f, 0.4f, 0.4f };
    cylinder.model.translation = { -1.5f, 0.0f, 2.1f };

    // Wall made of individual bricks
    mesh const brick_mesh = mesh_primitive_cube({ 0, 0, 0 }, 0.5f);

    int const rows = 4;
    int const cols = 5;
    float const spacing = 0.52f;

    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            mesh_drawable brick;
            brick.initialize_data_on_gpu(brick_mesh);
            brick.material.color = { 0.65f, 0.25f, 0.15f };

            float const offset = (row % 2) * spacing / 2.0f;

            brick.model.translation = { 0.0f,
                                        (col - (cols - 1) / 2.0f) * spacing
                                            + offset,
                                        -0.75f + row * spacing };

            bricks.push_back(brick);
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

    draw(ground, environment);
    draw(cylinder, environment);
    draw(sphere, environment);

    for (mesh_drawable const& brick : bricks)
        draw(brick, environment);
}

void scene_structure::display_gui()
{
    ImGui::Checkbox("Frame", &gui.display_frame);
    ImGui::SliderFloat("Time Scale", &timer.scale, 0.0f, 2.0f, "%.1f");
    ImGui::Checkbox("Wireframe", &gui.display_wireframe);
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
