cmake_minimum_required(VERSION 3.19...4.4)
project(project4_2026 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type")

find_package(Python 3.10 COMPONENTS Interpreter REQUIRED)
execute_process(
  COMMAND "${Python_EXECUTABLE}" -c "import sys; print(sys.prefix)"
  OUTPUT_VARIABLE PYTHON_PREFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE
  COMMAND_ERROR_IS_FATAL ANY)
list(PREPEND CMAKE_PREFIX_PATH "${PYTHON_PREFIX}")
find_package(ompl CONFIG REQUIRED)
message(STATUS "OMPL ${OMPL_VERSION}: ${ompl_DIR}")

add_executable(Project4Car
  src/Project4Car.cpp
  src/CollisionChecking.cpp
  src/RG-RRT.cpp)
add_executable(Project4Pendulum
  src/Project4Pendulum.cpp
  src/RG-RRT.cpp)
target_link_libraries(Project4Car PRIVATE ompl::ompl)
target_link_libraries(Project4Pendulum PRIVATE ompl::ompl)
set_target_properties(Project4Car Project4Pendulum PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
