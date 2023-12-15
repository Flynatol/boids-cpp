#include "ui.h"
#include "imgui.h"

#define NO_FONT_AWESOME
#include <rlImGui.h>

Ui::Ui() {
    rlImGuiSetup(true);
}

void Ui::Render(Camera2D &cam, Camera &camera, Rules &rules) {
    rlImGuiBegin();
        this->UpdateRulesWindow(rules);
        this->UpdatePerfMonitor();
        this->UpdateCameraWindow(camera);
        this->UpdateCameraWindow2d(cam);
    rlImGuiEnd();
}

void Ui::UpdateRulesWindow(Rules &rules) {
    ImGui::Begin("Boids Settings");
    ImGui::SliderFloat("alignment_factor", &rules.alignment_factor, 0., 1., "%0.9f");
    ImGui::SliderFloat("sight_range", &rules.sight_range, 0., 100., "%0.9f");
    ImGui::SliderFloat("avoid_distance_squared", &rules.avoid_distance_squared, 0., 1000., "%0.9f");
    ImGui::SliderFloat("avoid_factor", &rules.avoid_factor, 0., 1., "%0.9f");
    ImGui::SliderFloat("cohesion_factor", &rules.cohesion_factor, 0., 1., "%0.9f");
    ImGui::SliderFloat("rand", &rules.rand, 0., 1., "%0.9f");
    ImGui::SliderFloat("homing", &rules.homing, 0., 1., "%0.9f");
    ImGui::SliderInt("edge_width", &rules.edge_width, 0, 100, "%d");
    ImGui::SliderFloat("edge_factor", &rules.edge_factor, 0., 1., "%0.9f");
    ImGui::Checkbox("Show Debug Lines", &rules.show_lines);
    ImGui::End();

    rules.sight_range_squared = rules.sight_range * rules.sight_range;
}

void Ui::UpdatePerfMonitor() {
    ImGui::Begin("Performance Monitor");
    ImGui::Text("TPS: %d", GetFPS());
    ImGui::End();
}

void Ui::UpdateCameraWindow(Camera &camera){
    ImGui::Begin("Camera Settings");
    ImGui::SliderFloat("x", &camera.position.x, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("y", &camera.position.y, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("z", &camera.position.z, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("fovy", &camera.fovy, 0., 32., "%0.9f");

    ImGui::SliderFloat("x_target", &camera.target.x, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("y_target", &camera.target.y, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("z_target", &camera.target.z, -1000., 1000., "%0.9f");

    ImGui::End();
}

void Ui::UpdateCameraWindow2d(Camera2D &cam) {
    ImGui::Begin("Camera2d Settings");
    ImGui::SliderFloat("x_offset", &cam.offset.x, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("y_offset", &cam.offset.y, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("x_target", &cam.target.x, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("y_target", &cam.target.y, -1000., 1000., "%0.9f");
    ImGui::SliderFloat("zoom", &cam.zoom, 0., 32., "%0.9f");
    ImGui::End();
}