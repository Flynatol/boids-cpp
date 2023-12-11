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

// NOTE: Add here your custom variables

void main()
{
    /*
    // Texel color fetching from texture sampler
    vec4 texelColor = texture(texture0, fragTexCoord);
    vec3 lightDot = vec3(0.0);
    vec3 normal = normalize(fragNormal);
    vec3 viewD = normalize(viewPos - fragPosition);
    vec3 specular = vec3(0.0);
    */

    vec3 viewD = normalize(viewPos - fragPosition);
    finalColor = colDiffuse;// * vec4(1 - fragPosition, 1.0);
    finalColor = fragColor;
    //finalColor = vec4(length(viewD), 0., 0., 1.0);
    // Gamma correction 
    //finalColor = finalColor;
}
