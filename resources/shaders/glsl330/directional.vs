#version 330

// Input vertex attributes
in vec3 vertexPosition;

in float boid_x;
in float boid_y;
in float boid_vx;
in float boid_vy;

// Input uniform values
uniform mat4 mvp;

// Output vertex attributes (to fragment shader)
out vec3 fragPosition;
out vec4 fragColor;

void main() {
    float angle = atan(boid_vx, boid_vy);
    float sinres = sin(angle);
    float cosres = cos(angle);

    mat4 it2 = mat4(    cosres, 0.0, -sinres,0.0,  // 1. column
                        0.0,    1.0, 0.0,    0.0,  // 2. column
                        sinres, 0.0, cosres, 0.0,  // 3. column
                        boid_x, 0.0, boid_y, 1.0); // 4. column

    // Compute MVP for current instance
    mat4 mvpi = mvp*it2;
    
    // Send vertex attributes to fragment shader
    fragPosition = vec3(mvpi*vec4(vertexPosition, 1.0));
    fragColor = vec4((boid_vx+3)/6, boid_vy+3/6, 0.5, 1.0);

    // Calculate final vertex position
    gl_Position = mvpi*vec4(vertexPosition, 1.0);
}
