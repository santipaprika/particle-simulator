#pragma once

#include <vulkan/vulkan.h>

#include <Camera.hpp>

namespace vkr {
struct FrameInfo {
    int frameIndex;
    float frameTime;
    VkCommandBuffer commandBuffer;
    Camera& camera;
};
}  // namespace vkr