#include "app/app_setup.h"

#include <cstdlib>
#include <random>

#include "app/app_config.h"

Shader LoadBoidShader() {
    Shader boid_shader = LoadShader(
        TextFormat(RESOURCES_PATH "shaders/glsl%i/directional.vs", 330),
        TextFormat(RESOURCES_PATH "/shaders/glsl%i/simple.fs", 330)
    );

    boid_shader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(boid_shader, "viewPos");
    boid_shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocationAttrib(boid_shader, "instanceTransform");

    boid_shader.locs[SHADER_LOC_BOID_X] = GetShaderLocationAttrib(boid_shader, "boid_x");
    boid_shader.locs[SHADER_LOC_BOID_Y] = GetShaderLocationAttrib(boid_shader, "boid_y");
    boid_shader.locs[SHADER_LOC_BOID_VX] = GetShaderLocationAttrib(boid_shader, "boid_vx");
    boid_shader.locs[SHADER_LOC_BOID_VY] = GetShaderLocationAttrib(boid_shader, "boid_vy");

    DEBUG("boid_x at: %d", boid_shader.locs[SHADER_LOC_BOID_X]);
    DEBUG("viewPos at: %d", boid_shader.locs[SHADER_LOC_VECTOR_VIEW]);
    DEBUG("instanceTransform at: %d", boid_shader.locs[SHADER_LOC_MATRIX_MODEL]);

    return boid_shader;
}

Material CreateBoidMaterial(Shader boid_shader) {
    Material mat_instances = LoadMaterialDefault();
    mat_instances.shader = boid_shader;
    mat_instances.maps[MATERIAL_MAP_DIFFUSE].color = RED;
    return mat_instances;
}

Rules MakeDefaultRules() {
    return Rules {
        .avoid_distance_squared = 1000.f,
        .avoid_factor = 0.00000002f,
        .sight_range = SIGHT_RANGE,
        .sight_range_squared = SIGHT_RANGE * SIGHT_RANGE,
        .alignment_factor = 0.05f,
        .cohesion_factor = 0.0005f,
        .edge_width = 300,
        .edge_factor = 0.05f,
        .rand = 0.1f,
        .homing = 0.0000005f,
        .show_lines = false,
        .min_speed = 2,
        .max_speed = 3,
    };
}

void PopulateInitialBoids(BoidList& boid_list, const Rules& rules, float world_width, float world_height) {
    const float home_width = ((world_width - rules.edge_width * 2) / (16 + 1));
    const float home_height = ((world_height - rules.edge_width * 2) / (9 + 1));

    std::default_random_engine generator;
    std::uniform_real_distribution<float> width_distribution2(-home_width / 2.f, home_width / 2.f);
    std::uniform_real_distribution<float> height_distribution2(-home_height / 2.f, home_height / 2.f);

    for (int32_t i = 0; i < boid_list.m_size; i++) {
        boid_list.m_boid_store->index_next[i] = -1;
        boid_list.m_boid_store->vxs[i] = (std::rand() % 3) - 1;
        boid_list.m_boid_store->vys[i] = (std::rand() % 3) - 1;
        boid_list.m_boid_store->homes[i] = rand() % 144;

        int home_index_y = boid_list.m_boid_store->homes[i] / 16;
        int home_index_x = boid_list.m_boid_store->homes[i] % 16;

        boid_list.m_boid_store->xs[i] = (home_index_x + 1) * home_width + rules.edge_width + width_distribution2(generator);
        boid_list.m_boid_store->ys[i] = (home_index_y + 1) * home_height + rules.edge_width + height_distribution2(generator);
    }
}
