/*
*   Modified version of the framework provided by Brendan Galea in his Vulkan
*   tutorial series (https://github.com/blurrypiano/littleVulkanEngine) 
*   Copyright (c) 2020 Brendan Galea
*/

#pragma once

#include <Window.hpp>

// std lib headers
#include <string>
#include <vector>

namespace vkr {

struct SwapChainSupportDetails {
    VkSurfaceCapabilitiesKHR capabilities;
    std::vector<VkSurfaceFormatKHR> formats;
    std::vector<VkPresentModeKHR> presentModes;
};

struct QueueFamilyIndices {
    uint32_t graphicsFamily;
    uint32_t presentFamily;
    bool graphicsFamilyHasValue = false;
    bool presentFamilyHasValue = false;
    bool isComplete() { return graphicsFamilyHasValue && presentFamilyHasValue; }
};

class Device {
   public:
#ifdef NDEBUG
    const bool enableValidationLayers = false;
#else
    const bool enableValidationLayers = true;
#endif

    Device(Window &window);
    ~Device();

    // Not copyable or movable
    Device(const Device &) = delete;
    Device &operator=(const Device &) = delete;
    Device(Device &&) = delete;
    Device &operator=(Device &&) = delete;

    VkCommandPool getCommandPool() { return _commandPool; }
    VkDevice device() { return _device; }
    VkPhysicalDevice physicalDevice() { return _physicalDevice; }
    VkSurfaceKHR surface() { return _surface; }
    VkQueue graphicsQueue() { return _graphicsQueue; }
    VkQueue presentQueue() { return _presentQueue; }
    VkInstance instance() { return _instance; }
    QueueFamilyIndices queueFamilyIndices() { return _queueFamilyindices; }
    VkSampleCountFlagBits msaaSamples() { return _msaaSamples; }

    SwapChainSupportDetails getSwapChainSupport() { return querySwapChainSupport(_physicalDevice); }
    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties);
    QueueFamilyIndices findPhysicalQueueFamilies() { return findQueueFamilies(_physicalDevice); }
    VkFormat findSupportedFormat(
        const std::vector<VkFormat> &candidates, VkImageTiling tiling, VkFormatFeatureFlags features);

    // Buffer Helper Functions
    void createBuffer(
        VkDeviceSize size,
        VkBufferUsageFlags usage,
        VkMemoryPropertyFlags properties,
        VkBuffer &buffer,
        VkDeviceMemory &bufferMemory);
    VkCommandBuffer beginSingleTimeCommands();
    void endSingleTimeCommands(VkCommandBuffer commandBuffer);
    void transitionImageLayout(VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout, bool isCubemap = false);
    void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size, VkDeviceSize offset = 0, uint32_t regionCount = 1);
    void copyBufferToImage(
        VkBuffer buffer, VkImage image, uint32_t width, uint32_t height, uint32_t layerCount = 1);
    void copyBufferToCubemap(
        VkBuffer buffer, VkImage image, uint32_t width, uint32_t height, uint32_t faceSize, uint32_t layerCount = 1);

    void createImageWithInfo(
        const VkImageCreateInfo &imageInfo,
        VkMemoryPropertyFlags properties,
        VkImage &image,
        VkDeviceMemory &imageMemory);

    VkPhysicalDeviceProperties properties;
    void createCommandPool(VkCommandPool &commandPool, VkCommandPoolCreateFlags flags);

   private:
    void createInstance();
    void setupDebugMessenger();
    void createSurface();
    void pickPhysicalDevice();
    VkSampleCountFlagBits Device::getMaxUsableSampleCount();
    void createLogicalDevice();
    void createCommandPool();

    // helper functions
    bool isDeviceSuitable(VkPhysicalDevice device);
    std::vector<const char *> getRequiredExtensions();
    bool checkValidationLayerSupport();
    QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device);
    void populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT &createInfo);
    void hasGlfwRequiredInstanceExtensions();
    bool checkDeviceExtensionSupport(VkPhysicalDevice device);
    SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device);

    VkInstance _instance;
    VkDebugUtilsMessengerEXT _debugMessenger;
    VkPhysicalDevice _physicalDevice = VK_NULL_HANDLE;
    Window &_window;
    VkCommandPool _commandPool;

    VkDevice _device;
    VkSurfaceKHR _surface;
    QueueFamilyIndices _queueFamilyindices;
    VkQueue _graphicsQueue;
    VkQueue _presentQueue;
    VkSampleCountFlagBits _msaaSamples;

    const std::vector<const char *> _validationLayers = {"VK_LAYER_KHRONOS_validation"};
    const std::vector<const char *> _deviceExtensions = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
};

}  // namespace vkr
