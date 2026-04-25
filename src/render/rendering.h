#pragma once

#include <raylib.h>

class BoidList;
class Ui;
struct Rules;

Mesh GenMeshCustom();
void render(const BoidList& boid_list, Ui& ui, Rules& rules, Camera2D cam, Camera3D camera, Mesh& tri, Material& mat_instances);
Vector3 GetScreenToWorld3D(Vector2 position, Camera camera);
