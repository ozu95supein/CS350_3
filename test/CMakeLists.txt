cmake_minimum_required(VERSION 3.8)
project(cs350_test)

############################
# Testing
add_executable(${PROJECT_NAME}
			   common.hpp
			   common.cpp
			   bvh.cpp
#			   bvh_instructor.cpp
			   )
target_link_libraries(${PROJECT_NAME} PUBLIC cs350_engine)

# GTest
find_package(GTest CONFIG REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC GTest::gtest GTest::gtest_main) 



