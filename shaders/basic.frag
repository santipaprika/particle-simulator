#version 450

layout (location = 0) in vec3 fragColor;
layout (location = 1) in vec3 normalWS;
layout (location = 2) in vec2 fragTexCoord;

layout (location = 0) out vec4 outColor;

layout (binding = 0) uniform UniformBufferObject {
    mat4 projectionView;
    mat4 model;
    mat4 normalMatrix;
    vec3 camPos;
} ubo;

layout(binding = 1) uniform sampler2D texSampler;


const vec3 DIRECTION_TO_LIGHT = normalize(vec3(-1.0,-2.0,0.0));
vec3 AMBIENT = vec3(0.1);

layout(push_constant) uniform Push {
    float brightness;
} push;

void main() {
    vec3 lightIntensity = AMBIENT + vec3(max(dot(normalWS, DIRECTION_TO_LIGHT), 0));
    outColor = texture(texSampler,fragTexCoord*3) * vec4(fragColor * lightIntensity, 1.0);
}
