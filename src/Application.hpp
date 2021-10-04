/*
*   Modified version of the framework provided by Brendan Galea in his Vulkan
*   tutorial series (https://github.com/blurrypiano/littleVulkanEngine) 
*   Copyright (c) 2020 Brendan Galea
*/

#pragma once

#include <Device.hpp>
#include <Entity.hpp>
#include <Renderer.hpp>
#include <Scene.hpp>
#include <Window.hpp>

// std
#include <memory>
#include <vector>

namespace vkr {
class Application {
   public:
    static constexpr int WIDTH = 800;
    static constexpr int HEIGHT = 600;

    Application();
    ~Application();

    Application(const Application&) = delete;
    Application& operator=(const Application&) = delete;

    Device& getDevice() { return device; }
    Window& getWindow() { return window; }
    Renderer& getRenderer() { return renderer; }

    void run();

   private:
    Window window{WIDTH, HEIGHT, "Vulkan Tutorial"};
    Device device{window};
    Renderer renderer{window, device};

    Scene scene{device};
};
}  // namespace vkr
