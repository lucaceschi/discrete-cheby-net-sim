cmake_minimum_required(VERSION 3.26)
project(thesis-project-final)

set(EXTERNAL_DIR "${PROJECT_SOURCE_DIR}/external")
set(SOURCE_DIR "${PROJECT_SOURCE_DIR}/src")
set(DATA_DIR "${PROJECT_SOURCE_DIR}/data")
set(SIM_TARGET "sim")
set(CONVERTER_TARGET "mesh2vdb")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_FLAGS_RELEASE "-O2 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# jsoncpp
set(EXT_JSONCPP_DIR "${EXTERNAL_DIR}/jsoncpp")
set(JSONCPP_WITH_PKGCONFIG_SUPPORT OFF CACHE BOOL "" FORCE)
set(JSONCPP_WITH_CMAKE_PACKAGE OFF CACHE BOOL "" FORCE)
set(JSONCPP_WITH_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_OBJECT_LIBS OFF CACHE BOOL "" FORCE)
add_subdirectory(${EXT_JSONCPP_DIR})


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# OpenGL
include(FindOpenGL)


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# GLEW
set(EXT_GLEW_DIR "${EXTERNAL_DIR}/glew")
set(EXT_GLEW_INCLUDE_DIR "${EXT_GLEW_DIR}/include")
set(BUILD_UTILS OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(PROJECT_NAME "jsoncpp" CACHE STRING "" FORCE)
add_subdirectory("${EXT_GLEW_DIR}/build/cmake")
target_include_directories(glew_s PUBLIC
                           "$<BUILD_INTERFACE:${EXT_GLEW_INCLUDE_DIR}>"
                           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# GLFW
set(EXT_GLFW_DIR "${EXTERNAL_DIR}/glfw")
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
add_subdirectory(${EXT_GLFW_DIR})


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# imgui
set(EXT_IMGUI_DIR "${EXTERNAL_DIR}/imgui")
set(EXT_IMGUI_SOURCES
    "${EXT_IMGUI_DIR}/imgui.cpp"
    "${EXT_IMGUI_DIR}/imgui_draw.cpp"
    "${EXT_IMGUI_DIR}/imgui_widgets.cpp"
    "${EXT_IMGUI_DIR}/imgui_demo.cpp"
    "${EXT_IMGUI_DIR}/imgui_tables.cpp"
    "${EXT_IMGUI_DIR}/backends/imgui_impl_glfw.cpp"
    "${EXT_IMGUI_DIR}/backends/imgui_impl_opengl2.cpp"
    "${EXT_IMGUI_DIR}/backends/imgui_impl_opengl3.cpp"
)
add_library(imgui STATIC ${EXTERNAL_IMGUI_HEADERS} ${EXT_IMGUI_SOURCES})
target_include_directories(imgui PUBLIC
                           "$<BUILD_INTERFACE:${EXT_IMGUI_DIR};${EXT_IMGUI_DIR}/backends>"
                           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# VCGlib
set(EXT_VCGLIB_DIR "${EXTERNAL_DIR}/vcglib")
add_subdirectory(${EXT_VCGLIB_DIR})


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# OpenVDB
list(APPEND CMAKE_MODULE_PATH "/usr/lib64/cmake/OpenVDB/")
set(OpenVDB_INCLUDE_DIR "/usr/include/openvdb")
find_package(OpenVDB REQUIRED openvdb)


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# OpenMP
find_package(OpenMP)


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
set(CONVERTER_SOURCES
    "${SOURCE_DIR}/converter.cpp"
    "${EXTERNAL_DIR}/vcglib/wrap/ply/plylib.cpp"
)

add_executable(${CONVERTER_TARGET} ${CONVERTER_SOURCES})
target_include_directories(${CONVERTER_TARGET} PUBLIC ${SOURCE_DIR})

target_link_libraries(${CONVERTER_TARGET}
                      vcglib
                      OpenVDB::openvdb)


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
set(SIM_SOURCES
    "${SOURCE_DIR}/simulator.cpp"
    "${SOURCE_DIR}/simulator_app.cpp"
    "${SOURCE_DIR}/simulator_app_events.cpp"
    "${SOURCE_DIR}/simulator_app_gui.cpp"
    "${SOURCE_DIR}/simulator_app_init.cpp"
    "${SOURCE_DIR}/net.cpp"
    "${SOURCE_DIR}/forces.cpp"
    "${SOURCE_DIR}/constraints.cpp"
    "${SOURCE_DIR}/constraint_solver.cpp"
    "${SOURCE_DIR}/sdfs.cpp"
    "${SOURCE_DIR}/framework/app.cpp"
    "${SOURCE_DIR}/framework/debug.cpp"
    "${SOURCE_DIR}/framework/input.cpp"
    "${EXTERNAL_DIR}/vcglib/wrap/gui/trackball.cpp"
    "${EXTERNAL_DIR}/vcglib/wrap/gui/trackmode.cpp"
    "${EXTERNAL_DIR}/vcglib/wrap/ply/plylib.cpp"
)

add_executable(${SIM_TARGET} ${SIM_SOURCES})
target_include_directories(${SIM_TARGET} PUBLIC ${SOURCE_DIR})

target_link_libraries(${SIM_TARGET}
                      jsoncpp_static
                      ${OPENGL_glu_LIBRARY}
                      glew_s
                      glfw
                      imgui
                      vcglib
                      OpenVDB::openvdb
                      OpenMP::OpenMP_CXX)