#version 450

layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;
layout(location = 2) in vec3 normal;
layout(location = 3) in vec2 uv;

layout(location = 0) out vec4 fragColor;
layout(location = 1) out vec3 normalWS;
layout(location = 2) out vec2 fragTexCoord;
layout(location = 3) out vec3 positionWS;

layout (binding = 0) uniform UniformBufferObject {
    mat4 projectionView;
    mat4 model;
    mat4 normalMatrix;
    vec4 matColor;
    vec3 camPos;
} ubo;

void main() {
    positionWS = vec3(ubo.model * vec4(position, 1.0));
    gl_Position = ubo.projectionView * vec4(positionWS,1.f);

    normalWS = normalize(mat3(ubo.normalMatrix) * normal);
    fragColor = vec4(color,1.0) * ubo.matColor;
    fragTexCoord = uv;
}
