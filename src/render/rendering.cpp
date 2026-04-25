#include "render/rendering.h"

#include <glad/glad.h>
#include <raymath.h>

#include "app/app_config.h"
#include "sim/boidlist.h"
#include "rlgl.h"
#include "ui/ui.h"

namespace {

Matrix MatrixLookDown(Vector3 eye) {
    Matrix result = { 0 };

    result.m0 = 1.0f;
    result.m1 = 0.0f;
    result.m2 = 0.0f;
    result.m3 = 0.0f;
    result.m4 = 0.0f;
    result.m5 = 0.0f;
    result.m6 = 1.0f;
    result.m7 = 0.0f;
    result.m8 = 0.0f;
    result.m9 = -1.0f;
    result.m10 = 0.0f;
    result.m11 = 0.0f;
    result.m12 = -(eye.x);
    result.m13 = eye.z;
    result.m14 = -(eye.y);
    result.m15 = 1.0f;

    return result;
}

void fBeginMode3D(Camera camera) {
    rlMatrixMode(RL_PROJECTION);
    rlPushMatrix();
    rlLoadIdentity();

    float aspect = (float) GetScreenWidth() / (float) GetScreenHeight();

    double top = camera.fovy / 2.0;
    double right = top * aspect;

    rlOrtho(-right, right, -top, top, RL_CULL_DISTANCE_NEAR, RL_CULL_DISTANCE_FAR);

    rlMatrixMode(RL_MODELVIEW);
    rlLoadIdentity();

    Matrix matView = MatrixLookDown(camera.position);
    rlMultMatrixf(MatrixToFloat(matView));
}

unsigned int flLoadVertexBuffer(const void* buffer, int size, unsigned int prev_id) {
    if (!prev_id) {
        unsigned int id = 0;

        glGenBuffers(1, &id);
        glBindBuffer(GL_ARRAY_BUFFER, id);
        glBufferData(GL_ARRAY_BUFFER, size, buffer, GL_STREAM_DRAW);

        return id;
    }

    glBindBuffer(GL_ARRAY_BUFFER, prev_id);
    glBufferSubData(GL_ARRAY_BUFFER, 0, size, buffer);

    return prev_id;
}

unsigned int instances_boid_x_ID = 0;
unsigned int instances_boid_y_ID = 0;
unsigned int instances_boid_vx_ID = 0;
unsigned int instances_boid_vy_ID = 0;

void DrawMeshInstanced2(Mesh mesh, Material material, int instances, float* boid_x, float* boid_y, float* boid_vx, float* boid_vy) {
    FrameMarkStart("Begin Render");
#define MAX_MATERIAL_MAPS 12

    rlEnableShader(material.shader.id);

    unsigned int id = 0;
    glGenBuffers(1, &id);

    if (material.shader.locs[SHADER_LOC_COLOR_DIFFUSE] != -1) {
        float values[4] = {
            (float) material.maps[MATERIAL_MAP_DIFFUSE].color.r / 255.0f,
            (float) material.maps[MATERIAL_MAP_DIFFUSE].color.g / 255.0f,
            (float) material.maps[MATERIAL_MAP_DIFFUSE].color.b / 255.0f,
            (float) material.maps[MATERIAL_MAP_DIFFUSE].color.a / 255.0f
        };

        rlSetUniform(material.shader.locs[SHADER_LOC_COLOR_DIFFUSE], values, SHADER_UNIFORM_VEC4, 1);
    }

    rlEnableVertexArray(mesh.vaoId);

    FrameMarkEnd("Begin Render");
    FrameMarkStart("Upload");

    instances_boid_x_ID = flLoadVertexBuffer(boid_x, instances * sizeof(float), instances_boid_x_ID);
    rlEnableVertexAttribute(material.shader.locs[SHADER_LOC_BOID_X]);
    rlSetVertexAttribute(material.shader.locs[SHADER_LOC_BOID_X], 1, RL_FLOAT, false, sizeof(float), 0);
    rlSetVertexAttributeDivisor(material.shader.locs[SHADER_LOC_BOID_X], 1);

    instances_boid_y_ID = flLoadVertexBuffer(boid_y, instances * sizeof(float), instances_boid_y_ID);
    rlEnableVertexAttribute(material.shader.locs[SHADER_LOC_BOID_Y]);
    rlSetVertexAttribute(material.shader.locs[SHADER_LOC_BOID_Y], 1, RL_FLOAT, false, sizeof(float), 0);
    rlSetVertexAttributeDivisor(material.shader.locs[SHADER_LOC_BOID_Y], 1);

    instances_boid_vx_ID = flLoadVertexBuffer(boid_vx, instances * sizeof(float), instances_boid_vx_ID);
    rlEnableVertexAttribute(material.shader.locs[SHADER_LOC_BOID_VX]);
    rlSetVertexAttribute(material.shader.locs[SHADER_LOC_BOID_VX], 1, RL_FLOAT, false, sizeof(float), 0);
    rlSetVertexAttributeDivisor(material.shader.locs[SHADER_LOC_BOID_VX], 1);

    instances_boid_vy_ID = flLoadVertexBuffer(boid_vy, instances * sizeof(float), instances_boid_vy_ID);
    rlEnableVertexAttribute(material.shader.locs[SHADER_LOC_BOID_VY]);
    rlSetVertexAttribute(material.shader.locs[SHADER_LOC_BOID_VY], 1, RL_FLOAT, false, sizeof(float), 0);
    rlSetVertexAttributeDivisor(material.shader.locs[SHADER_LOC_BOID_VY], 1);

    FrameMarkEnd("Upload");
    FrameMarkStart("End Render");

    Matrix matModelView = MatrixMultiply(rlGetMatrixTransform(), rlGetMatrixModelview());
    rlEnableVertexArray(mesh.vaoId);

    Matrix matModelViewProjection = MatrixMultiply(matModelView, rlGetMatrixProjection());
    rlSetUniformMatrix(material.shader.locs[SHADER_LOC_MATRIX_MVP], matModelViewProjection);

    rlDrawVertexArrayInstanced(0, mesh.vertexCount, instances);

    for (int i = 0; i < MAX_MATERIAL_MAPS; i++) {
        if (material.maps[i].texture.id > 0) {
            rlActiveTextureSlot(i);

            if ((i == MATERIAL_MAP_IRRADIANCE) ||
                (i == MATERIAL_MAP_PREFILTER) ||
                (i == MATERIAL_MAP_CUBEMAP)) {
                rlDisableTextureCubemap();
            } else {
                rlDisableTexture();
            }
        }
    }

    FrameMarkEnd("End Render");
}

} // namespace

Mesh GenMeshCustom() {
    Mesh mesh = { 0 };
    mesh.triangleCount = 1;
    mesh.vertexCount = mesh.triangleCount * 3;
    mesh.vertices = (float*) MemAlloc(mesh.vertexCount * 3 * sizeof(float));
    mesh.colors = (unsigned char*) MemAlloc(mesh.vertexCount * 4 * sizeof(unsigned char));

    mesh.vertices[0] = -TRIANGLE_SIZE;
    mesh.vertices[1] = 0.;
    mesh.vertices[2] = -2 * TRIANGLE_SIZE;

    mesh.vertices[3] = 0.;
    mesh.vertices[4] = 0.;
    mesh.vertices[5] = 2 * TRIANGLE_SIZE;

    mesh.vertices[6] = TRIANGLE_SIZE;
    mesh.vertices[7] = 0.;
    mesh.vertices[8] = -2 * TRIANGLE_SIZE;

    mesh.colors[0] = 255;
    mesh.colors[1] = 0;
    mesh.colors[2] = 0;
    mesh.colors[3] = 255;

    mesh.colors[4] = 0;
    mesh.colors[5] = 255;
    mesh.colors[6] = 0;
    mesh.colors[7] = 255;

    mesh.colors[8] = 0;
    mesh.colors[9] = 0;
    mesh.colors[10] = 255;
    mesh.colors[11] = 255;

    UploadMesh(&mesh, false);

    return mesh;
}

void render(const BoidList& boid_list, Ui& ui, Rules& rules, Camera2D cam, Camera3D camera, Mesh& tri, Material& mat_instances) {
    ZoneScoped;
    BeginDrawing();
        ClearBackground(BLACK);

        auto const boid_store = boid_list.m_backbuffer;
        int offset = 0;

        fBeginMode3D(camera);
            DrawMeshInstanced2(tri, mat_instances, boid_list.m_size - offset, boid_store->xs + offset, boid_store->ys + offset, boid_store->vxs + offset, boid_store->vys + offset);
        EndMode3D();

        ui.Render(cam, camera, rules);
    EndDrawing();
}

Vector3 GetScreenToWorld3D(Vector2 position, Camera camera) {
    Matrix invMatCamera = MatrixInvert(GetCameraMatrix(camera));
    return Vector3Transform(Vector3 { position.x, position.y, 0 }, invMatCamera);
}
