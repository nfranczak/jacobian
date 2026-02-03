cmake_minimum_required(VERSION 3.16)
project(jacobian VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Find or fetch tinyxml2
find_package(tinyxml2 QUIET)
if(NOT tinyxml2_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        tinyxml2
        GIT_REPOSITORY https://github.com/leethomason/tinyxml2.git
        GIT_TAG 10.0.0
    )
    FetchContent_MakeAvailable(tinyxml2)
endif()

# Library
add_library(jacobian
    src/urdf_parser.cpp
    src/forward_kinematics.cpp
    src/jacobian.cpp
)

target_include_directories(jacobian
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(jacobian
    PUBLIC
        Eigen3::Eigen
    PRIVATE
        tinyxml2::tinyxml2
)

# Test executable
add_executable(test_jacobian test/test_jacobian.cpp)
target_link_libraries(test_jacobian PRIVATE jacobian)

# Copy URDFs to build directory for testing
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/urdfs DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
