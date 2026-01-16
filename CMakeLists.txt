cmake_minimum_required(VERSION 3.16)
project(LBM_Flow_Simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fetch SFML from GitHub
include(FetchContent)

set(FETCHCONTENT_QUIET FALSE)
set(BUILD_SHARED_LIBS FALSE)
set(SFML_BUILD_AUDIO FALSE)
set(SFML_BUILD_NETWORK FALSE)

FetchContent_Declare(
        SFML
        GIT_REPOSITORY https://github.com/SFML/SFML.git
        GIT_TAG 2.6.1
        GIT_PROGRESS TRUE
)

message(STATUS "Downloading and building SFML...")
FetchContent_MakeAvailable(SFML)

# Source files (main.cpp in root directory)
add_executable(${PROJECT_NAME} main.cpp)

# Link SFML
target_link_libraries(${PROJECT_NAME} PRIVATE
        sfml-graphics
        sfml-window
        sfml-system
)

# Platform-specific settings
if(WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE sfml-main)
endif()

message(STATUS "Configuration complete. Run 'cmake --build .' to build the project.")