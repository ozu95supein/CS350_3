cmake_minimum_required(VERSION 3.8)
project(cs350_engine)

# Engine library
add_library(${PROJECT_NAME}
			bvh_indexed.hpp
			bvh_indexed.cpp
			# ...
			)
target_include_directories(${PROJECT_NAME} PUBLIC .)

# GLFW3
find_package(glfw3 CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC glfw)

# GLAD
find_package(glad CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC glad::glad)

# GLM
find_package(glm CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC glm::glm)

# Lodepng
find_package(lodepng CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC lodepng)
