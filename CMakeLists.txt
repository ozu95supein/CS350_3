cmake_minimum_required(VERSION 3.8)
project(cs350_bvh)

############################
# Compiler options (for all targets)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (MSVC)
	add_compile_options(/W3)
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
else ()
	add_compile_options(-Wall -Wextra -pedantic)
endif ()

# For VSCODE
set(CMAKE_EXPORT_COMPILE_COMMANDS ON) 

add_subdirectory(src)
add_subdirectory(imgui)
add_subdirectory(demo)
add_subdirectory(test)