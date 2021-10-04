cmake_minimum_required(VERSION 3.0)


# set build dir for debug and release configs
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG "${CMAKE_SOURCE_DIR}/build/debug")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE "${CMAKE_SOURCE_DIR}/build/release")

include_directories("vulkan" "animation" "${CMAKE_CURRENT_LIST_DIR}")
file(GLOB project_src CONFIGURE_DEPENDS "*.hpp" "*.cpp" "vulkan/*.hpp" "vulkan/*.cpp" "animation/*.hpp" "animation/*.cpp")

add_executable( ${PROJECT_NAME} ${project_src} )

target_link_libraries( ${PROJECT_NAME} Vulkan::Vulkan glfw glm stb_image imgui)
