/*
*   Modified version of the framework provided by Brendan Galea in his Vulkan
*   tutorial series (https://github.com/blurrypiano/littleVulkanEngine)
*   Copyright (c) 2020 Brendan Galea
*/

#include <Application.hpp>
#include <FrameInfo.hpp>
#include <ImGuiHelper.hpp>
#include <InputController.hpp>
#include <RenderSystem.hpp>
#include <Utils.hpp>

// libs
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>

// std
#include <array>
#include <cassert>
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace vkr {

Application::Application() { scene.initialize(); }

Application::~Application() {
}

void Application::run() {
    RenderSystem renderSystem{device, renderer.getSwapChainRenderPass(), scene};
    ImGuiHelper imGuiHelper(*this);

    auto viewerObject = Entity::createEntity();
    viewerObject.transform.translation = glm::vec3(0,3,-3);
    viewerObject.transform.rotation = glm::vec3(PI_2/2.f,0.f,0.f);
    InputController cameraController{};

    bool useMSAA = true;
    bool switchedMSAA = false;

    auto currentTime = std::chrono::high_resolution_clock::now();
    while (!window.shouldClose()) {
        glfwPollEvents();

        ImGui_ImplVulkan_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("App window");
        ImGui::Checkbox("Use Skybox", &scene.getMainCamera().hasSkybox());
        switchedMSAA = ImGui::Checkbox("Use MSAA", &useMSAA);

        ImGui::End();

        // ImGui::ShowDemoWindow();
        ImGui::ShowMetricsWindow();
        ImGui::Render();

        auto newTime = std::chrono::high_resolution_clock::now();
        float frameTime =
            std::chrono::duration<float, std::chrono::seconds::period>(newTime - currentTime).count();
        currentTime = newTime;

        cameraController.moveInPlaneXZ(window.getGLFWwindow(), frameTime, viewerObject);
        scene.getMainCamera().update(viewerObject.transform, renderer.getAspectRatio());
        scene.updateScene(frameTime, renderSystem.getDescriptorPool(), renderSystem.getDescriptorSetLayout());

        if (auto commandBuffer = renderer.beginFrame()) {
            FrameInfo frameInfo{renderer.getFrameIndex(), frameTime, commandBuffer, scene.getMainCamera()};

            renderer.beginCommandBuffer(commandBuffer);
            renderer.beginSwapChainRenderPass(commandBuffer);

            renderSystem.renderEntities(frameInfo);

            renderer.endSwapChainRenderPass(commandBuffer);
            renderer.endCommandBuffer(commandBuffer);

            VkCommandBuffer imGuiCommandBuffer = imGuiHelper.renderImGui();

            bool wasWindowResized = false;
            renderer.endFrame(std::vector<VkCommandBuffer>({commandBuffer, imGuiCommandBuffer}), wasWindowResized);

            if (wasWindowResized) {
                imGuiHelper.updateImGuiFramebuffers();
            }
        }

        // If MSAA is enabled/disabled recreate Swapchain and Pipelines (3->2 / 2->3 images)
        if (switchedMSAA) {
            renderer.recreateSwapChain(useMSAA);
            renderSystem.recreatePipelines(renderer.getSwapChainRenderPass(), useMSAA);
            imGuiHelper.recreate();
        }
    }

    vkDeviceWaitIdle(device.device());
}

}  // namespace vkr
