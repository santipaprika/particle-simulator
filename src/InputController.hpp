/*
*   Modified version of the framework provided by Brendan Galea in his Vulkan
*   tutorial series (https://github.com/blurrypiano/littleVulkanEngine) 
*   Copyright (c) 2020 Brendan Galea
*/

#pragma once

#include <Entity.hpp>
#include <Window.hpp>

namespace vkr {
class InputController {
   public:
    struct KeyMappings {
        int moveLeft = GLFW_KEY_A;
        int moveRight = GLFW_KEY_D;
        int moveForward = GLFW_KEY_W;
        int moveBackward = GLFW_KEY_S;
        int moveUp = GLFW_KEY_E;
        int moveDown = GLFW_KEY_Q;
        int lookLeft = GLFW_KEY_LEFT;
        int lookRight = GLFW_KEY_RIGHT;
        int lookUp = GLFW_KEY_UP;
        int lookDown = GLFW_KEY_DOWN;
        int moveSphereLeft = GLFW_KEY_J;
        int moveSphereRight = GLFW_KEY_L;
        int moveSphereForward = GLFW_KEY_I;
        int moveSphereBackward = GLFW_KEY_K;
        int moveSphereUp = GLFW_KEY_O;
        int moveSphereDown = GLFW_KEY_U;
    };

    void moveInPlaneXZ(GLFWwindow* window, float dt, Entity& entity);
    void moveEntityInPlaneXZ(GLFWwindow* window, float dt, Entity& entity);

    KeyMappings keys{};
    float moveSpeed{3.f};
    float lookSpeed{1.5f};
};
}  // namespace vkr