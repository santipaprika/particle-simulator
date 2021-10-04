/*
*   Modified version of the framework provided by Brendan Galea in his Vulkan
*   tutorial series (https://github.com/blurrypiano/littleVulkanEngine) 
*   Copyright (c) 2020 Brendan Galea
*/

#include <stb_image.h>

#include <Entity.hpp>
#include <FrameInfo.hpp>

namespace vkr {

glm::mat4 TransformComponent::mat4() {
    const float c3 = glm::cos(rotation.z);
    const float s3 = glm::sin(rotation.z);
    const float c2 = glm::cos(rotation.x);
    const float s2 = glm::sin(rotation.x);
    const float c1 = glm::cos(rotation.y);
    const float s1 = glm::sin(rotation.y);
    return glm::mat4{
        {
            scale.x * (c1 * c3 + s1 * s2 * s3),
            scale.x * (c2 * s3),
            scale.x * (c1 * s2 * s3 - c3 * s1),
            0.0f,
        },
        {
            -scale.y * (c3 * s1 * s2 - c1 * s3),
            -scale.y * (c2 * c3),
            -scale.y * (c1 * c3 * s2 + s1 * s3),
            0.0f,
        },
        {
            -scale.z * (c2 * s1),
            -scale.z * (-s2),
            -scale.z * (c1 * c2),
            0.0f,
        },
        {translation.x, translation.y, translation.z, 1.0f}};
}

glm::mat3 TransformComponent::normalMatrix() {
    const float c3 = glm::cos(rotation.z);
    const float s3 = glm::sin(rotation.z);
    const float c2 = glm::cos(rotation.x);
    const float s2 = glm::sin(rotation.x);
    const float c1 = glm::cos(rotation.y);
    const float s1 = glm::sin(rotation.y);
    const glm::vec3 invScale = 1.0f / scale;

    return glm::mat3{
        {
            invScale.x * (c1 * c3 + s1 * s2 * s3),
            invScale.x * (c2 * s3),
            invScale.x * (c1 * s2 * s3 - c3 * s1),
        },
        {
            invScale.y * (c3 * s1 * s2 - c1 * s3),
            invScale.y * (c2 * c3),
            invScale.y * (c1 * c3 * s2 + s1 * s3),
        },
        {
            invScale.z * (c2 * s1),
            invScale.z * (-s2),
            invScale.z * (c1 * c2),
        },
    };
}

void Entity::update(float dt) {
    if (particle)
        transform.translation = particle->updateInScene(dt);
}

void Entity::render(glm::mat4 camProjectionView, FrameInfo& frameInfo, VkPipelineLayout pipelineLayout) {
    auto modelMatrix = transform.mat4();
    EntityUBO entityUBO = {camProjectionView, modelMatrix, transform.normalMatrix(), frameInfo.camera.getPosition()};

    uboBuffers[frameInfo.frameIndex]->writeToBuffer(&entityUBO);
    uboBuffers[frameInfo.frameIndex]->flush();

    VkCommandBuffer commandBuffer = frameInfo.commandBuffer;
    SimplePushConstantData push{0.1f};
    vkCmdPushConstants(
        commandBuffer,
        pipelineLayout,
        VK_SHADER_STAGE_FRAGMENT_BIT,
        0,
        sizeof(SimplePushConstantData),
        &push);

    mesh->bind(commandBuffer);
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSet, 0, nullptr);
    mesh->draw(commandBuffer);
}

}  // namespace vkr