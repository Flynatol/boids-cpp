#pragma once

#include "sim/boidlist.h"
#include "ui/ui.h"

Shader LoadBoidShader();
Material CreateBoidMaterial(Shader boid_shader);
Rules MakeDefaultRules();
void PopulateInitialBoids(BoidList& boid_list, const Rules& rules, float world_width, float world_height);
