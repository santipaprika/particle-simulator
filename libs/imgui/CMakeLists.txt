file(GLOB imgui-src CONFIGURE_DEPENDS "*.h" "*.cpp")

add_library(imgui ${imgui-src})

target_include_directories(imgui INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  )
target_link_libraries( imgui Vulkan::Vulkan glfw )