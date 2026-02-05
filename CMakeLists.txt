cmake_minimum_required(VERSION 3.10)
project(A03 C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------- Vulkan ----------
find_package(Vulkan REQUIRED)

# ---------- glslangValidator (from Vulkan SDK) ----------
find_program(GLSLANG_VALIDATOR
        NAMES glslangValidator glslangValidator.exe
        HINTS "$ENV{VULKAN_SDK}/Bin" "$ENV{VULKAN_SDK}/Bin32"
        REQUIRED
)
message(STATUS "glslangValidator: ${GLSLANG_VALIDATOR}")

# ---------- Dependencies (GLFW + GLM) ----------
include(FetchContent)

FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.4
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
        glm
        GIT_REPOSITORY https://github.com/g-truc/glm.git
        GIT_TAG 1.0.1
)
FetchContent_MakeAvailable(glm)

# ---------- Project sources ----------
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE HEADERS CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/include/*.h" "${CMAKE_SOURCE_DIR}/include/*.hpp")

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
target_include_directories(${PROJECT_NAME} PRIVATE "${CMAKE_SOURCE_DIR}/include")
target_link_libraries(${PROJECT_NAME} PRIVATE Vulkan::Vulkan glfw glm::glm)

# ---------- Shaders ----------
set(SHADERS_SRC_DIR "${CMAKE_SOURCE_DIR}/shaders")
set(SHADERS_OUT_DIR "${CMAKE_BINARY_DIR}/shaders")
file(MAKE_DIRECTORY "${SHADERS_OUT_DIR}")

file(GLOB GLSL_SOURCE_FILES CONFIGURE_DEPENDS
        "${SHADERS_SRC_DIR}/*.vert"
        "${SHADERS_SRC_DIR}/*.frag"
)

set(SPIRV_BINARY_FILES "")
foreach(GLSL ${GLSL_SOURCE_FILES})
    get_filename_component(FILE_NAME ${GLSL} NAME)  # keeps .vert/.frag
    set(SPV_FILE "${SHADERS_OUT_DIR}/${FILE_NAME}.spv")

    add_custom_command(
            OUTPUT "${SPV_FILE}"
            COMMAND "${GLSLANG_VALIDATOR}" -V "${GLSL}" -o "${SPV_FILE}"
            DEPENDS "${GLSL}"
            COMMENT "Compiling shader: ${GLSL}"
            VERBATIM
    )
    list(APPEND SPIRV_BINARY_FILES "${SPV_FILE}")
endforeach()

add_custom_target(Shaders DEPENDS ${SPIRV_BINARY_FILES})
add_dependencies(${PROJECT_NAME} Shaders)

# ---------- Runtime layout: copy next to exe ----------
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${PROJECT_NAME}>/shaders"
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${SHADERS_OUT_DIR}" "$<TARGET_FILE_DIR:${PROJECT_NAME}>/shaders"
        COMMENT "Copying shaders next to the executable..."
        VERBATIM
)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:${PROJECT_NAME}>/assets"
        COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_SOURCE_DIR}/assets" "$<TARGET_FILE_DIR:${PROJECT_NAME}>/assets"
        COMMENT "Copying assets next to the executable..."
        VERBATIM
)
