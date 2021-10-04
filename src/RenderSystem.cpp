/*
*   Modified version of the framework provided by Brendan Galea in his Vulkan
*   tutorial series (https://github.com/blurrypiano/littleVulkanEngine) 
*   Copyright (c) 2020 Brendan Galea
*/

#include <Hair.hpp>
#include <RenderSystem.hpp>
#include <SwapChain.hpp>

// libs
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

// std
#include <array>
#include <cassert>
#include <stdexcept>

namespace vkr {

RenderSystem::RenderSystem(Device& device, VkRenderPass renderPass, Scene& scene, bool useMSAA)
    : device{device}, scene{scene} {
    createUniformBuffers();
    setupDescriptors();

    createPipelineLayout();
    createPipeline(renderPass, useMSAA);
}

void RenderSystem::setupDescriptors() {
    createDescriptorSetLayout();

    uint32_t entitiesCount = static_cast<uint32_t>(scene.getEntities().size() + 1);  // +1 from skybox
    std::vector<PoolSize> poolSizes = {PoolSize{VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, entitiesCount},
                                       PoolSize{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, entitiesCount}};
    createDescriptorPool(poolSizes, entitiesCount);

    createDescriptorSets();
}

RenderSystem::~RenderSystem() {
    // Pipeline is deleted implicitly because of unique_ptr out of scope after instance destruction.

    vkDestroyPipelineLayout(device.device(), pipelineLayout, nullptr);
    vkDestroyDescriptorSetLayout(device.device(), descriptorSetLayout, nullptr);
    vkDestroyDescriptorPool(device.device(), descriptorPool, nullptr);

    for (auto& entity : scene.getEntities()) {
        entity.material->getAlbedo()->destroy();
    }

    if (scene.getMainCamera().hasSkybox())
        scene.getMainCamera().getSkybox().material->getAlbedo()->destroy();
}

void RenderSystem::createPipelineLayout() {
    VkPushConstantRange pushConstantRange{};
    pushConstantRange.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    pushConstantRange.offset = 0;
    pushConstantRange.size = sizeof(SimplePushConstantData);

    VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
    pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutInfo.setLayoutCount = 1;
    pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;
    pipelineLayoutInfo.pushConstantRangeCount = 1;
    pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

    if (vkCreatePipelineLayout(device.device(), &pipelineLayoutInfo, nullptr, &pipelineLayout) !=
        VK_SUCCESS) {
        throw std::runtime_error("failed to create pipeline layout!");
    }
}

void RenderSystem::createPipeline(VkRenderPass renderPass, bool useMSAA) {
    // 3 pipelines at the moment:
    //     1. Triangular mesh
    //     2. Hair (Line strip)
    //     3. Skybox

    assert(pipelineLayout != nullptr && "Cannot create pipeline before pipeline layout");
    PipelineConfigInfo pipelineConfig{};
    Pipeline::defaultPipelineConfigInfo(pipelineConfig, device, useMSAA);
    pipelineConfig.renderPass = renderPass;
    pipelineConfig.pipelineLayout = pipelineLayout;

    PipelineConfigInfo hairPipelineConfig = pipelineConfig;
    hairPipelineConfig.inputAssemblyInfo.topology = VK_PRIMITIVE_TOPOLOGY_LINE_STRIP;
    hairPipelineConfig.inputAssemblyInfo.primitiveRestartEnable = VK_TRUE;

    PipelineConfigInfo skyboxPipelineConfig = pipelineConfig;
    skyboxPipelineConfig.rasterizationInfo.cullMode = VK_CULL_MODE_BACK_BIT;

    ShaderPaths basicShaderPaths;
    basicShaderPaths.fragFilepath = "../shaders/basic.frag.spv";
    basicShaderPaths.vertFilepath = "../shaders/basic.vert.spv";

    ShaderPaths hairShaderPaths;
    hairShaderPaths.fragFilepath = "../shaders/hair.frag.spv";
    hairShaderPaths.vertFilepath = "../shaders/hair.vert.spv";

    ShaderPaths skyboxShaderPaths;
    skyboxShaderPaths.fragFilepath = "../shaders/skybox.frag.spv";
    skyboxShaderPaths.vertFilepath = "../shaders/skybox.vert.spv";

    std::vector<ShaderPaths> pipelinesShaderPaths = {basicShaderPaths, hairShaderPaths, skyboxShaderPaths};

    VertexInputDescriptions meshPipelineInputDescriptions;
    meshPipelineInputDescriptions.attributeDescription = Mesh::Vertex::getAttributeDescriptions();
    meshPipelineInputDescriptions.bindingDescription = Mesh::Vertex::getBindingDescriptions();

    VertexInputDescriptions hairPipelineInputDescriptions;
    hairPipelineInputDescriptions.attributeDescription = Hair::Vertex::getAttributeDescriptions();
    hairPipelineInputDescriptions.bindingDescription = Hair::Vertex::getBindingDescriptions();

    // Skybox pipeline uses same description as triangle mesh pipeline.

    pipelines = Pipeline::createGraphicsPipelines(
        device,
        pipelinesShaderPaths,
        std::vector<PipelineConfigInfo>({pipelineConfig, hairPipelineConfig, skyboxPipelineConfig}),
        std::vector<VertexInputDescriptions>({meshPipelineInputDescriptions, hairPipelineInputDescriptions, meshPipelineInputDescriptions}));
}

void RenderSystem::createDescriptorSetLayout() {
    std::array<VkDescriptorSetLayoutBinding, 2> setLayoutBindings{};

    // Binding 0: Uniform buffers (used to pass transforms)
    setLayoutBindings[0].binding = 0;
    setLayoutBindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    setLayoutBindings[0].descriptorCount = 1;
    setLayoutBindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

    // Binding 1: Combined image sampler (used to pass per object texture information)
    setLayoutBindings[1].binding = 1;
    setLayoutBindings[1].descriptorCount = 1;
    setLayoutBindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    setLayoutBindings[1].pImmutableSamplers = nullptr;
    setLayoutBindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = static_cast<uint32_t>(setLayoutBindings.size());
    layoutInfo.pBindings = setLayoutBindings.data();

    if (vkCreateDescriptorSetLayout(device.device(), &layoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS) {
        throw std::runtime_error("failed to create descriptor set layout!");
    }
}

void RenderSystem::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(EntityUBO);
    VkDeviceSize lightBufferSize = sizeof(EntityUBO);

    // Triangle meshes
    for (auto& entity : scene.getEntities()) {
        entity.uboBuffers.resize(SwapChain::MAX_FRAMES_IN_FLIGHT);
        for (auto& uboBuffer : entity.uboBuffers) {
            uboBuffer = std::make_unique<Buffer>(device, bufferSize, 1,
                                                 VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
                                                 device.properties.limits.minUniformBufferOffsetAlignment);
            uboBuffer->map();
        }
    }

    // Lights
    for (auto& lightEntity : scene.getLightEntities()) {
        // Create light specific UBO
        lightEntity.uboBuffers.resize(SwapChain::MAX_FRAMES_IN_FLIGHT);
        for (auto& uboBuffer : lightEntity.uboBuffers) {
            uboBuffer = std::make_unique<Buffer>(device, lightBufferSize, 1,
                                                 VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
                                                 device.properties.limits.minUniformBufferOffsetAlignment);
            uboBuffer->map();
        }
    }

    // Skybox
    if (scene.getMainCamera().hasSkybox()) {
        scene.getMainCamera().getSkybox().uboBuffers.resize(SwapChain::MAX_FRAMES_IN_FLIGHT);
        for (auto& uboBuffer : scene.getMainCamera().getSkybox().uboBuffers) {
            uboBuffer = std::make_unique<Buffer>(device, bufferSize, 1,
                                                 VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT,
                                                 device.properties.limits.minUniformBufferOffsetAlignment);
            uboBuffer->map();
        }
    }
}

void RenderSystem::createDescriptorPool(const std::vector<PoolSize>& poolSizes, int maxSets) {
    std::vector<VkDescriptorPoolSize> descriptorPoolSizes(poolSizes.size());

    for (int i = 0; i < poolSizes.size(); i++) {
        descriptorPoolSizes[i].type = poolSizes[i].type;
        descriptorPoolSizes[i].descriptorCount = poolSizes[i].descriptorCount;
    }

    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.poolSizeCount = static_cast<uint32_t>(descriptorPoolSizes.size());
    poolInfo.pPoolSizes = descriptorPoolSizes.data();
    poolInfo.maxSets = maxSets;

    if (vkCreateDescriptorPool(device.device(), &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS) {
        throw std::runtime_error("failed to create descriptor pool!");
    }
}

void RenderSystem::createDescriptorSets() {
    for (auto& entity : scene.getEntities()) {
        updateDescriptorSet(entity);
    }
    if (scene.getMainCamera().hasSkybox())
        updateDescriptorSet(scene.getMainCamera().getSkybox());
}

void RenderSystem::updateDescriptorSet(Entity& entity) {
    // Allocates an empty descriptor set without actual descriptors from the pool using the set layout
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool;
    // This is for each entity. Even if there are two bindings per object, these are contained in a single descriptor.
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &descriptorSetLayout;

    if (vkAllocateDescriptorSets(device.device(), &allocInfo, &entity.descriptorSet) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate descriptor sets!");
    }

    // Update the descriptor set with the actual descriptors matching shader bindings set in the layout

    // Binding 0: Object matrices uniform buffer
    std::array<VkWriteDescriptorSet, 2> descriptorWrites{};
    descriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrites[0].dstSet = entity.descriptorSet;
    descriptorWrites[0].dstBinding = 0;
    descriptorWrites[0].dstArrayElement = 0;

    descriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    descriptorWrites[0].descriptorCount = 1;

    descriptorWrites[0].pBufferInfo = &entity.uboBuffers[0]->descriptorInfo(sizeof(EntityUBO));

    // Binding 1: Object texture
    descriptorWrites[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    descriptorWrites[1].dstSet = entity.descriptorSet;
    descriptorWrites[1].dstBinding = 1;
    descriptorWrites[1].dstArrayElement = 0;
    descriptorWrites[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    descriptorWrites[1].descriptorCount = 1;

    descriptorWrites[1].pImageInfo = &entity.material->getAlbedo()->getDescriptorInfo();

    // Binding 2: Lights

    vkUpdateDescriptorSets(device.device(), static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
}

void RenderSystem::renderEntities(FrameInfo frameInfo) {
    auto projectionView = frameInfo.camera.getProjection() * frameInfo.camera.getView();

    VkCommandBuffer commandBuffer = frameInfo.commandBuffer;

    // TRIANGULAR MESHES
    pipelines->meshes->bind(commandBuffer);
    for (auto& entity : scene.getEntities()) {
        if (!entity.mesh) continue;
        entity.render(projectionView, frameInfo, pipelineLayout);
    }

    // HAIR (LINES)
    pipelines->hair->bind(commandBuffer);
    for (auto& entity : scene.getEntities()) {
        if (!entity.hair) continue;

        auto modelMatrix = entity.transform.mat4();
        EntityUBO entityUBO = {projectionView, modelMatrix, entity.transform.normalMatrix(), frameInfo.camera.getPosition()};

        entity.uboBuffers[frameInfo.frameIndex]->writeToBuffer(&entityUBO);
        entity.uboBuffers[frameInfo.frameIndex]->flush();

        SimplePushConstantData push{0.1f};
        vkCmdPushConstants(
            commandBuffer,
            pipelineLayout,
            VK_SHADER_STAGE_FRAGMENT_BIT,
            0,
            sizeof(SimplePushConstantData),
            &push);

        entity.hair->bind(commandBuffer);
        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &entity.descriptorSet, 0, nullptr);
        entity.hair->draw(commandBuffer);
    }

    // SKYBOX
    if (scene.getMainCamera().hasSkybox()) {
        pipelines->skybox->bind(commandBuffer);
        scene.getMainCamera().getSkybox().render(projectionView, frameInfo, pipelineLayout);
    }
}

void RenderSystem::recreatePipelines(VkRenderPass renderPass, bool useMSAA) {
    createPipeline(renderPass, useMSAA);
}

}  // namespace vkr
