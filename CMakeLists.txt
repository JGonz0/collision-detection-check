cmake_minimum_required(VERSION 3.17)

project(circle)
add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} GL GLU glut)
