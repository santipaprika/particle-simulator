#version 450

layout (location = 0) in vec3 fragColor;
layout (location = 1) in vec3 directionWS;
layout (location = 2) in vec3 positionWS;

layout (location = 0) out vec4 outColor;

layout (binding = 0) uniform UniformBufferObject {
    mat4 projectionView;
    mat4 model;
    mat4 normalMatrix;
    vec3 camPos;
} ubo;

layout(binding = 1) uniform sampler2D texSampler;

layout(push_constant) uniform Push {
    float brightness;
} push;

const vec3 DIRECTION_TO_LIGHT = normalize(vec3(3.0, -3.0, -1.0));
vec3 AMBIENT = vec3(0.1);



vec4 getAmbientAndDiffuse(vec4 lightColor0, vec4 diffuseColor, vec3 N, vec3 L)
{
    return (lightColor0 * diffuseColor * clamp(dot(N, L),0.0,1.0) + vec4(AMBIENT,1.0));
}

vec4 getSpecular(vec4 lightColor0, vec4 specularColor, vec3 N, vec3 T, vec3 V, vec3 L, float specPower, float specScale)
{
    vec4 specular = vec4(0.0, 0.0, 0.0, 0.0);
    vec3 H = normalize(V + L);
    float HdotT = clamp(dot(T, H),0.0,1.0);
    float sinTH = sqrt(1 - HdotT * HdotT);

    return specularColor * clamp(pow(sinTH, specPower),0.0,1.0) * specScale;
}

void main() {
    
    vec3 L = DIRECTION_TO_LIGHT;
    vec3 V = normalize(ubo.camPos - positionWS);
    vec3 T = normalize(directionWS);
    vec3 VTPlaneNormal = normalize(cross(V, T));
    vec3 N = normalize(cross(T, VTPlaneNormal));

    vec4 diffuse = getAmbientAndDiffuse(vec4(1.0), vec4(fragColor,1.0), N, L);
    vec4 specular = getSpecular(vec4(1.0), vec4(1.0), N, T, V, L, 0.1, 0.1);

    outColor = diffuse + specular;
}
