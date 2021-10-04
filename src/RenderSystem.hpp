/*
*   Modified version of the framework provided by Brendan Galea in his Vulkan
*   tutorial series (https://github.com/blurrypiano/littleVulkanEngine) 
*   Copyright (c) 2020 Brendan Galea
*/

#pragma once

#include <Camera.hpp>
#include <Device.hpp>
#include <Entity.hpp>
#include <Pipeline.hpp>
#include <FrameInfo.hpp>
#include <Scene.hpp>

// std
#include <memory>
#include <vector>

namespace vkr {

struct PoolSize {
    VkDescriptorType type;
    uint32_t descriptorCount;
};

class RenderSystem {
   public:
    RenderSystem(Device &device, VkRenderPass renderPass, Scene &scene, bool useMSAA = true);
    ~RenderSystem();

    RenderSystem(const RenderSystem &) = delete;
    RenderSystem &operator=(const RenderSystem &) = delete;

    void setupDescriptors();

    void renderEntities(FrameInfo frameInfo);
    void recreatePipelines(VkRenderPass renderPass, bool useMSAA = true);

   private:
    void createDescriptorSetLayout();
    void createUniformBuffers();
    void createDescriptorPool(const std::vector<PoolSize> &poolSizes, int maxSets);
    void createDescriptorSets();

    void createPipelineLayout();
    void createPipeline(VkRenderPass renderPass, bool useMSAA = true);

    void updateDescriptorSet(Entity& entity);

    Device &device;

    std::unique_ptr<PipelineSet> pipelines;

    VkDescriptorSetLayout descriptorSetLayout;
    VkPipelineLayout pipelineLayout;

    Scene& scene;

    VkDescriptorPool descriptorPool;

    void *data;
};
}  // namespace vkr
