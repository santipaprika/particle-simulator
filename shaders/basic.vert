#version 450

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;
layout(location = 2) in vec3 normal;
layout(location = 3) in vec2 uv;

layout(location = 0) out vec3 fragColor;
layout(location = 1) out vec3 normalWS;
layout(location = 2) out vec2 fragTexCoord;

layout (binding = 0) uniform UniformBufferObject {
    mat4 projectionView;
    mat4 model;
    mat4 normalMatrix;
    vec3 camPos;
} ubo;

void main() {
    vec4 positionWS = ubo.model * vec4(position, 1.0);
    gl_Position = ubo.projectionView * positionWS;

    normalWS = normalize(mat3(ubo.normalMatrix) * normal);
    fragColor = color;
    fragTexCoord = uv;
}
