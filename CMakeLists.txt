cmake_minimum_required(VERSION 3.17)

project(circle)
add_executable(${PROJECT_NAME} main.cpp)
target_link_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/../lib/)
target_link_libraries(circle freeglut opengl32 glu32)
