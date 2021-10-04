set(stb_image_src 
    ${CMAKE_CURRENT_SOURCE_DIR}/stb_image.h
    ${CMAKE_CURRENT_SOURCE_DIR}/stb_image.cc
)

add_library(stb_image ${stb_image_src})

target_include_directories(stb_image INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)