#version 330

// Input vertex attributes (from vertex shader)
in vec3 fragPosition;
in vec3 vertexPosition2;
in vec4 fragColor;

// Input uniform values
uniform vec4 colDiffuse;
uniform vec3 viewPos;

// Output fragment color
out vec4 finalColor;

void main()
{
    //vec3 viewD = normalize(viewPos - fragPosition);
    //finalColor = colDiffuse;// * vec4(1 - fragPosition, 1.0);
    finalColor = fragColor;
}
