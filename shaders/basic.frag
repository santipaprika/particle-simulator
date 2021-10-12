#version 450

layout (location = 0) in vec4 fragColor;
layout (location = 1) in vec3 normalWS;
layout (location = 2) in vec2 fragTexCoord;

layout (location = 0) out vec4 outColor;

layout (binding = 0) uniform UniformBufferObject {
    mat4 projectionView;
    mat4 model;
    mat4 normalMatrix;
    vec4 matColor;
    vec3 camPos;
} ubo;

layout(binding = 1) uniform sampler2D texSampler;


const vec3 DIRECTION_TO_LIGHT = normalize(vec3(-1.0,-1.0,0.0));
const vec3 DIRECTION_TO_LIGHT2 = normalize(vec3(0.1,0.2,1.0));
vec4 AMBIENT = vec4(vec3(0.1),1.0);

layout(push_constant) uniform Push {
    float brightness;
} push;

void main() {
    vec4 lightIntensity = AMBIENT + vec4(vec3(max(dot(normalWS, DIRECTION_TO_LIGHT), 0)), 1.0) + vec4(vec3(max(dot(normalWS, DIRECTION_TO_LIGHT2), 0)), 1.0) ;
    outColor = texture(texSampler,fragTexCoord) * fragColor * lightIntensity;

}
