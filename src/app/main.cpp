#include <algorithm>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdint>

#include <raylib.h>
#include <raymath.h>

#include "app/app_config.h"
#include "app/app_setup.h"
#include "app/sim_dump.h"
#include "render/rendering.h"
#include "sim/simulation.h"
#include "ui/ui.h"

#pragma comment(lib, "Winmm.lib")

int main(int argc, char* argv[]) {
    uint32_t num_boids = (argc > 1) ? atoi(argv[1]) : 2000000;
    SimDumpWriter sim_dump(argc, argv);

    SetTraceLogLevel(LOG_ALL);

    InitWindow(0, 0, "RayLib Boids!");
    SetTargetFPS(FRAME_RATE_LIMIT);

    const int32_t screen_width = GetScreenWidth();
    const int32_t screen_height = GetScreenHeight();

    DEBUG("Using %d args", argc);
    DEBUG("Using %d threads", num_threads);

    TraceLog(LOG_DEBUG, TextFormat("Boid size is: %d bytes", sizeof(Boid)));

    Mesh tri = GenMeshCustom();
    Shader boid_shader = LoadBoidShader();
    Material mat_instances = CreateBoidMaterial(boid_shader);
    Rules rules = MakeDefaultRules();

    Ui ui;

    const int32_t world_size_mult = (uint32_t) std::ceil(
        sqrt((BOID_DENSITY_MAGIC_NUMBER / ((double) screen_height)) * ((double) num_boids / (double) screen_width))
    );

    const float world_width = (float) (screen_width * world_size_mult);
    const float world_height = (float) (screen_height * world_size_mult);

    Camera2D cam = {
        .zoom = (float) screen_width / (float) world_width,
    };

    Camera camera = {
        .position = Vector3 {world_width / 2.f, 200.f, world_height / 2.f},
        .target = Vector3 {world_width / 2.f, 0.0f, world_height / 2.f},
        .up = Vector3 { 0.0f, 0.0f, -1.0f },
        .fovy = (float) world_height,
        .projection = CAMERA_ORTHOGRAPHIC,
    };

    BoidList boid_list_local(num_boids);
    BoidMap boid_map_local(world_height, world_width, CELL_WIDTH);

    boid_list = &boid_list_local;
    boid_map = &boid_map_local;

    const bool avx512_compiled = boids_avx512_compiled();
    const bool avx512_runtime = avx512_compiled && boids_cpu_supports_avx512();
    TraceLog(
        LOG_INFO,
        TextFormat(
            "SIMD path: AVX2 base, AVX-512 compiled=%s runtime=%s",
            avx512_compiled ? "yes" : "no",
            avx512_runtime ? "enabled" : "disabled"
        )
    );
    sim_dump.log_configuration();

    PopulateInitialBoids(*boid_list, rules, world_width, world_height);

    std::memcpy(boid_list->m_backbuffer->xs, boid_list->m_boid_store->xs, sizeof(float) * boid_list->m_size);
    std::memcpy(boid_list->m_backbuffer->ys, boid_list->m_boid_store->ys, sizeof(float) * boid_list->m_size);
    std::memcpy(boid_list->m_backbuffer->vxs, boid_list->m_boid_store->vxs, sizeof(float) * boid_list->m_size);
    std::memcpy(boid_list->m_backbuffer->vys, boid_list->m_boid_store->vys, sizeof(float) * boid_list->m_size);
    std::memcpy(boid_list->m_backbuffer->homes, boid_list->m_boid_store->homes, sizeof(int32_t) * boid_list->m_size);

    const uint32_t task_size = 10000;
    const uint32_t num_tasks = (boid_list->m_size + (task_size - 1)) / task_size;
    populate_args* args_populate = new populate_args[num_tasks];

    auto args_update = new row_runner_args[boid_map->m_ysize];
    auto args_rebuild = new rebuild_args[boid_map->m_ysize];

    Boid* index_buffer = (Boid*) malloc(boid_map->m_xsize * boid_map->m_ysize * sizeof(Boid));

    for (uint32_t i = 0; i < boid_map->m_ysize; i++) {
        args_update[i] = row_runner_args {
            .y = i,
            .rules = &rules,
        };

        args_rebuild[i] = rebuild_args {
            .y = i,
            .index_buffer = index_buffer,
        };
    }

    DEBUG("Num tasks: %d", num_tasks);

    for (uint32_t i = 0; i < num_tasks; i++) {
        args_populate[i] = populate_args {
            .start = i * task_size,
            .task_size = std::min(task_size, boid_list->m_size - (i * task_size)),
            .rebuild_args = args_rebuild,
        };
    }
    args_populate[0].num_tasks = num_tasks;

    BoidTaskMaster task_master;
    task_master.start_threads();

    TaskSync task_populate;
    populate_map2(&task_master, &task_populate, args_populate, num_tasks);
    task_populate.wait();

    TaskSync task_update;

    TaskSync task_rebuild;
    rebuild_list2(args_rebuild, &task_master, &task_rebuild);
    task_rebuild.wait();

    while (WindowShouldClose() == false) {
        auto t_update_start = TIME_NOW;
        const double frame_time_seconds = GetFrameTime();

        if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
            Vector2 delta = Vector2Scale(GetMouseDelta(), -1.0f / cam.zoom);

            cam.target = Vector2Add(cam.target, delta);

            camera.position = Vector3 {camera.position.x + delta.x, camera.position.y, camera.position.z + delta.y };
            camera.target = Vector3 {camera.target.x + delta.x, camera.target.y, camera.target.z + delta.y };
        }

        if (float wheel = GetMouseWheelMove()) {
            Vector2 mouseWorldPos = GetScreenToWorld2D(GetMousePosition(), cam);
            Vector3 mouseWorldPos2 = GetScreenToWorld3D(GetMousePosition(), camera);

            DEBUG("OG: [X: %f, Y: %f] NEW: [X: %f, Y: %f, Z: %f]", mouseWorldPos.x, mouseWorldPos.y, mouseWorldPos2.x, mouseWorldPos2.y, mouseWorldPos2.z)

            auto m_pos = GetMousePosition();

            cam.offset = m_pos;
            cam.target = mouseWorldPos;
            cam.zoom += wheel * 0.125f;

            if (cam.zoom < (1. / world_size_mult)) cam.zoom = (1. / world_size_mult);

            camera.fovy = screen_height / cam.zoom;

            camera.position = Vector3 {
                .x = (screen_width * 0.5f - m_pos.x) / cam.zoom + mouseWorldPos.x,
                .y = camera.position.y,
                .z = (screen_height * 0.5f - m_pos.y) / cam.zoom + mouseWorldPos.y,
            };

            camera.target = Vector3 {camera.position.x, 0.f, camera.position.z};
        }

        float cameraPos[3] = { camera.position.x, camera.position.y, camera.position.z };
        SetShaderValue(boid_shader, boid_shader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);

        FrameMarkStart("wait_rebuild");
        task_rebuild.wait();
        FrameMarkEnd("wait_rebuild");

        sim_dump.tick(frame_time_seconds, *boid_list, *boid_map, world_width, world_height);

        update_boids2(args_update, &task_master, &task_update);

        auto t_start_drawing = TIME_NOW;
        FrameMarkStart("Render");
        render(*boid_list, ui, rules, cam, camera, tri, mat_instances);
        FrameMarkEnd("Render");
        auto t_end_drawing = TIME_NOW;

        task_update.wait();

        rebuild_list2(args_rebuild, &task_master, &task_rebuild);

        auto t_update_end = TIME_NOW;

        DEBUG("Rebuild: na, render: %0.4f, comp: %0.4f", std::chrono::duration<double, std::milli>(t_end_drawing - t_start_drawing).count(), std::chrono::duration<double, std::milli>(t_update_end - t_update_start).count());

        FrameMark;
    }

    task_master.queue_stop_all();
    task_master.join_all();

    delete[] args_populate;
    delete[] args_update;
    delete[] args_rebuild;
    free(index_buffer);

    UnloadMaterial(mat_instances);
    UnloadShader(boid_shader);
    UnloadMesh(tri);

    rl_CloseWindow();
    return 0;
}
