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

layout (location = 3) out vec3 UVW;

void main() {
    vec4 positionWS = ubo.model * vec4(position, 1.0);
    vec4 pos = ubo.projectionView * positionWS;
    
    UVW = position;
    // Convert cubemap coordinates into Vulkan coordinate space
	UVW.xy *= -1.0;

    // z = w to fake furthest depth
    gl_Position = vec4(pos.xy, pos.w-0.00001, pos.w);

    normalWS = normalize(mat3(ubo.normalMatrix) * normal);
    fragColor = color;
    fragTexCoord = uv;
}
