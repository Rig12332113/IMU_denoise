cmake_minimum_required(VERSION 3.10)

project(IMU_denoise LANGUAGES CXX)

find_package(nlohmann_json REQUIRED)
add_executable(IMU_server ./src/receiver.cpp)

target_link_libraries(IMU_server
    PRIVATE nlohmann_json::nlohmann_json
)
