/*
*   Modified version of the framework provided by Brendan Galea in his Vulkan
*   tutorial series (https://github.com/blurrypiano/littleVulkanEngine) 
*   Copyright (c) 2020 Brendan Galea
*/

#include <Renderer.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

// std
#include <array>
#include <cassert>
#include <stdexcept>

namespace vkr {

Renderer::Renderer(Window &window, Device &device)
    : window{window}, device{device} {
    recreateSwapChain();
    createCommandBuffers();
}

Renderer::~Renderer() { freeCommandBuffers(); }

void Renderer::recreateSwapChain(bool useMSAA) {
    auto extent = window.getExtent();
    while (extent.width == 0 || extent.height == 0) {
        extent = window.getExtent();
        glfwWaitEvents();
    }
    vkDeviceWaitIdle(device.device());

    if (swapChain == nullptr) {
        swapChain = std::make_unique<SwapChain>(device, extent, useMSAA);
    } else {
        std::shared_ptr<SwapChain> oldSwapChain = std::move(swapChain);
        swapChain = std::make_unique<SwapChain>(device, extent, oldSwapChain, useMSAA);

        if (!oldSwapChain->compareSwapFormats(*swapChain.get())) {
            throw std::runtime_error("Swap chain image(or depth) format has changed!");
        }
    }
}

void Renderer::createCommandBuffers(std::vector<VkCommandBuffer> &commandBuffers, VkCommandPool commandPool, uint32_t commandBufferCount) {
    commandBuffers.resize(commandBufferCount);

    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandPool = commandPool;
    allocInfo.commandBufferCount = commandBufferCount;

    if (vkAllocateCommandBuffers(device.device(), &allocInfo, commandBuffers.data()) !=
        VK_SUCCESS) {
        throw std::runtime_error("failed to allocate command buffers!");
    }
}

void Renderer::createCommandBuffers() {
    createCommandBuffers(commandBuffers, device.getCommandPool());
}

void Renderer::freeCommandBuffers() {
    vkFreeCommandBuffers(
        device.device(),
        device.getCommandPool(),
        static_cast<uint32_t>(commandBuffers.size()),
        commandBuffers.data());
    commandBuffers.clear();
}

VkCommandBuffer Renderer::beginFrame() {
    assert(!isFrameStarted && "Can't call beginFrame while already in progress");

    if (!acquireNextSwapChainImage()) {
        return nullptr;
    }

    isFrameStarted = true;

    return getCurrentCommandBuffer();
}

void Renderer::endFrame(std::vector<VkCommandBuffer> commandBuffers, bool &wasWindowResized) {
    assert(isFrameStarted && "Can't call endFrame while frame is not in progress");

    auto result = swapChain->submitCommandBuffers(commandBuffers, &currentImageIndex);
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR ||
        window.wasWindowResized()) {
        window.resetWindowResizedFlag();
        recreateSwapChain();
        wasWindowResized = true;

        // upon recreating the swapchain, the minimum amount of image views might have changed
        ImGui_ImplVulkan_SetMinImageCount(device.getSwapChainSupport().capabilities.minImageCount + 1);

    } else if (result != VK_SUCCESS) {
        throw std::runtime_error("failed to present swap chain image!");
    }

    isFrameStarted = false;
    currentFrameIndex = (currentFrameIndex + 1) % SwapChain::MAX_FRAMES_IN_FLIGHT;
}

void Renderer::beginCommandBuffer(VkCommandBuffer commandBuffer) {
    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

    if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS) {
        throw std::runtime_error("failed to begin recording command buffer!");
    }
}

void Renderer::endCommandBuffer(VkCommandBuffer commandBuffer) {
    if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
        throw std::runtime_error("failed to record command buffer!");
    }
}

void Renderer::beginSwapChainRenderPass(VkCommandBuffer &commandBuffer,
                                        VkRenderPass renderPass,
                                        VkFramebuffer framebuffer,
                                        const std::vector<VkClearValue> &clearValues,
                                        bool isFirstPass) {
    if (isFirstPass) {
        assert(isFrameStarted && "Can't call beginSwapChainRenderPass if frame is not in progress");
        assert(
            commandBuffer == getCurrentCommandBuffer() &&
            "Can't begin render pass on command buffer from a different frame");
    }

    VkRenderPassBeginInfo renderPassInfo{};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    renderPassInfo.renderPass = renderPass;
    renderPassInfo.framebuffer = framebuffer;

    renderPassInfo.renderArea.offset = {0, 0};
    renderPassInfo.renderArea.extent = swapChain->getSwapChainExtent();

    renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
    renderPassInfo.pClearValues = clearValues.data();

    vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport viewport{};
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = static_cast<float>(swapChain->getSwapChainExtent().width);
    viewport.height = static_cast<float>(swapChain->getSwapChainExtent().height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    VkRect2D scissor{{0, 0}, swapChain->getSwapChainExtent()};
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
}

void Renderer::beginSwapChainRenderPass(VkCommandBuffer &commandBuffer) {
    beginSwapChainRenderPass(commandBuffer,
                             swapChain->getRenderPass(),
                             swapChain->getFrameBuffer(currentImageIndex),
                             std::vector<VkClearValue>({{0.01f, 0.01f, 0.01f, 1.0f}, {1.0f, 0.f}}));
}

void Renderer::endSwapChainRenderPass(VkCommandBuffer commandBuffer, bool isFirstPass) {
    if (isFirstPass) {
        assert(isFrameStarted && "Can't call endSwapChainRenderPass if frame is not in progress");
        assert(
            commandBuffer == getCurrentCommandBuffer() &&
            "Can't end render pass on command buffer from a different frame");
    }
    vkCmdEndRenderPass(commandBuffer);
}

bool Renderer::acquireNextSwapChainImage() {
    auto result = swapChain->acquireNextImage(&currentImageIndex);
    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
        recreateSwapChain();
        return false;
    }

    if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
        throw std::runtime_error("failed to acquire swap chain image!");
    }

    return true;
}

}  // namespace vkr
