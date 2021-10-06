/*
*   Modified version of the framework provided by Brendan Galea in his Vulkan
*   tutorial series (https://github.com/blurrypiano/littleVulkanEngine) 
*   Copyright (c) 2020 Brendan Galea
*/

#include <stb_image.h>

#include <Entity.hpp>
#include <FrameInfo.hpp>
#include <SwapChain.hpp>
#include <stdexcept>

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

// If update returns false the entity should be removed from the scene
bool Entity::update(float dt) {
    if (particle) {
        transform.translation = particle->updateInScene(dt);    // this updates lifetime too
        return !particle->isDead();
    }

    return true;
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

void Entity::createUniformBuffer(Device& device) {
    VkDeviceSize bufferSize = sizeof(EntityUBO);
    uboBuffers.resize(SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (auto& uboBuffer : uboBuffers) {
        uboBuffer = std::make_unique<Buffer>(device, bufferSize, 1,
                                             VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
                                             device.properties.limits.minUniformBufferOffsetAlignment);
        uboBuffer->map();
    }
}

void Entity::updateDescriptorSet(Device& device, VkDescriptorPool& descriptorPool, VkDescriptorSetLayout& descriptorSetLayout) {
    // Allocates an empty descriptor set without actual descriptors from the pool using the set layout
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool;
    // This is for each entity. Even if there are two bindings per object, these are contained in a single descriptor.
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &descriptorSetLayout;

    if (vkAllocateDescriptorSets(device.device(), &allocInfo, &descriptorSet) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate descriptor sets!");
    }

    // Update the descriptor set with the actual descriptors matching shader bindings set in the layout

    // Binding 0: Object matrices uniform buffer
    std::array<VkWriteDescriptorSet, 2> descriptorWrites{};
    descriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrites[0].dstSet = descriptorSet;
    descriptorWrites[0].dstBinding = 0;
    descriptorWrites[0].dstArrayElement = 0;

    descriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    descriptorWrites[0].descriptorCount = 1;

    descriptorWrites[0].pBufferInfo = &uboBuffers[0]->descriptorInfo(sizeof(EntityUBO));

    // Binding 1: Object texture
    descriptorWrites[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrites[1].dstSet = descriptorSet;
    descriptorWrites[1].dstBinding = 1;
    descriptorWrites[1].dstArrayElement = 0;
    descriptorWrites[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    descriptorWrites[1].descriptorCount = 1;

    descriptorWrites[1].pImageInfo = &material->getAlbedo()->getDescriptorInfo();

    // Binding 2: Lights

    vkUpdateDescriptorSets(device.device(), static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
}

}  // namespace vkr