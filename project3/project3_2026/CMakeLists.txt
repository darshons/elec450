cmake_minimum_required(VERSION 3.19...4.4)
project(project3_2026 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")

find_package(Python 3.10 COMPONENTS Interpreter Development.Module REQUIRED)
execute_process(
  COMMAND "${Python_EXECUTABLE}" -c "import sys, nanobind; print(sys.prefix); print(nanobind.cmake_dir())"
  OUTPUT_VARIABLE PYTHON_PREFIXES
  OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY)
string(REPLACE "\n" ";" PYTHON_PREFIXES "${PYTHON_PREFIXES}")
list(PREPEND CMAKE_PREFIX_PATH ${PYTHON_PREFIXES})
find_package(nanobind CONFIG REQUIRED)
find_package(ompl CONFIG REQUIRED)
message(STATUS "OMPL ${OMPL_VERSION}: ${ompl_DIR}")

nanobind_add_module(project3_cpp
  provided/bindings.cpp
  provided/ArmPlanning.cpp
  provided/CollisionChecking3D.cpp
  src/RTP.cpp)
target_include_directories(project3_cpp PRIVATE provided src)
target_link_libraries(project3_cpp PRIVATE ompl::ompl)
set_target_properties(project3_cpp PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
