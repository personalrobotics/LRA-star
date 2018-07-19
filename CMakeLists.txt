cmake_minimum_required(VERSION 2.8.3)
project(LRA-star)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package(catkin REQUIRED
  COMPONENTS
  cmake_modules)
find_package(OMPL REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread filesystem graph program_options system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

link_libraries(
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_library(${PROJECT_NAME}
  src/LRAstar.cpp
  src/utils/BisectPerm.cpp
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# For example scripts
add_executable(example2D scripts/example2D.cpp)
target_link_libraries(example2D ${PROJECT_NAME})

# This custom target doesn't affect building the project but is only for
# displaying the header files in IDEs.
FILE(GLOB_RECURSE LibFiles "include/*.hpp")
add_custom_target(headers SOURCES ${LibFiles})

#===============================================================================
# Code Formatting
#===============================================================================
if(LRASTAR_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Code Formatting ]")
endif()

find_program(
  CLANG_FORMAT_EXECUTABLE
  NAMES clang-format-3.8 clang-format
)

get_property(formatting_files GLOBAL PROPERTY LRASTAR_FORMAT_FILES)
list(LENGTH formatting_files formatting_files_length)
message(STATUS "Formatting on ${formatting_files_length} source files.")

if (CLANG_FORMAT_EXECUTABLE)

  if(LRASTAR_VERBOSE)
    message(STATUS "Looking for clang-format - ${CLANG_FORMAT_EXECUTABLE} found")
  endif()

  add_custom_target(format
                    COMMAND ${CMAKE_COMMAND} -E echo "Formatting code... "
                    COMMAND ${CLANG_FORMAT_EXECUTABLE} -style=file -i ${formatting_files}
                    COMMAND ${CMAKE_COMMAND} -E echo "Done."
                    DEPENDS ${CLANG_FORMAT_EXECUTABLE}
                    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/include/LRAstar)

  add_custom_target(check-format
                    COMMAND ${CMAKE_COMMAND} -E echo "Checking code format... "
                    COMMAND ${CMAKE_SOURCE_DIR}/tools/check_format.sh ${CLANG_FORMAT_EXECUTABLE} ${formatting_files}
                    COMMAND ${CMAKE_COMMAND} -E echo "Done."
                    DEPENDS ${CLANG_FORMAT_EXECUTABLE}
                    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/include/LRAstar)

else()

  message(STATUS "Looking for clang-format - NOT found, please install clang-format to enable automatic code formatting")

endif()
