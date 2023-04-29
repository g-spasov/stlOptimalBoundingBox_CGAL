# Created by the script cgal_create_cmake_script
# This is the CMake script for compiling a CGAL application.

cmake_minimum_required(VERSION 3.1...3.23)
project(Optimal_bounding_box)

find_package(CGAL REQUIRED)

set(CGAL_DATA_DIR "/home/gspasov/CGAL-5.5.1/data")



find_package(Eigen3 3.1.0 REQUIRED) #(3.1.0 or greater)
include(CGAL_Eigen3_support)
if(NOT TARGET CGAL::Eigen3_support)
  message(
    STATUS "This project requires the Eigen library, and will not be compiled.")
  return()
endif()

create_single_source_cgal_program("optimalBoundingBox.cpp")
create_single_source_cgal_program("capStlOfOBB.cpp")
set_property(TARGET optimalBoundingBox PROPERTY CXX_STANDARD 17)
set_property(TARGET capStlOfOBB PROPERTY CXX_STANDARD 17)

foreach(target optimalBoundingBox)
  target_link_libraries(${target} PUBLIC CGAL::Eigen3_support)
endforeach()

foreach(target capStlOfOBB)
  target_link_libraries(${target} PUBLIC CGAL::Eigen3_support)
endforeach()