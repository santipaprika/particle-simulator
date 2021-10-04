/*
*   Modified version of the framework provided by Brendan Galea in his Vulkan
*   tutorial series (https://github.com/blurrypiano/littleVulkanEngine) 
*   Copyright (c) 2020 Brendan Galea
*/

#pragma once

#include <Device.hpp>
#include <SwapChain.hpp>
#include <Window.hpp>

// std
#include <cassert>
#include <memory>
#include <vector>

namespace vkr {
class Renderer {
   public:
    Renderer(Window &window, Device &device);
    ~Renderer();

    Renderer(const Renderer &) = delete;
    Renderer &operator=(const Renderer &) = delete;

    VkRenderPass getSwapChainRenderPass() const { return swapChain->getRenderPass(); }
    float getAspectRatio() const { return swapChain->extentAspectRatio(); }
    uint32_t getSwapChainImageCount() const { return static_cast<uint32_t>(swapChain->imageCount()); }
    std::shared_ptr<SwapChain> getSwapChain() { return swapChain; }
    bool isFrameInProgress() const { return isFrameStarted; }

    VkCommandBuffer getCurrentCommandBuffer(bool requireSync = true) const {
        if (requireSync)
            assert(isFrameStarted && "Cannot get command buffer when frame not in progress");

        return commandBuffers[currentFrameIndex];
    }

    int getFrameIndex() const {
        assert(isFrameStarted && "Cannot get frame index when frame not in progress");
        return currentFrameIndex;
    }
    int getImageIndex() const { return currentImageIndex; }

    VkCommandBuffer beginFrame();
    void endFrame(std::vector<VkCommandBuffer> commandBuffers, bool &wasWindowResized);
    void beginCommandBuffer(VkCommandBuffer commandBuffer);
    void endCommandBuffer(VkCommandBuffer commandBuffer);
    void beginSwapChainRenderPass(VkCommandBuffer &commandBuffer,
                                  VkRenderPass renderPass,
                                  VkFramebuffer framebuffer,
                                  const std::vector<VkClearValue> &clearValues,
                                  bool isFirstPass = true);
    void beginSwapChainRenderPass(VkCommandBuffer &commandBuffer);
    void endSwapChainRenderPass(VkCommandBuffer commandBuffer, bool isFirstPass = true);
    bool acquireNextSwapChainImage();
    void createCommandBuffers(std::vector<VkCommandBuffer> &commandBuffers,
                              VkCommandPool commandPool,
                              uint32_t commandBufferCount = static_cast<uint32_t>(SwapChain::MAX_FRAMES_IN_FLIGHT));
    void recreateSwapChain(bool useMSAA = true);

   private:
    void createCommandBuffers();
    void freeCommandBuffers();

    Window &window;
    Device &device;
    std::shared_ptr<SwapChain> swapChain;
    std::vector<VkCommandBuffer> commandBuffers;

    uint32_t currentImageIndex;
    int currentFrameIndex{0};
    bool isFrameStarted{false};
};
}  // namespace vkr
