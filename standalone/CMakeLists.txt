# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/standalone/include)

# Library source files
set(LIB_SOURCES
    src/lsm6dso32.cpp
)

# Create shared library
add_library(lsm6dso32 SHARED ${LIB_SOURCES})
target_link_libraries(lsm6dso32 pthread)

# Test program
add_executable(test_imu examples/test_imu.cpp)
target_link_libraries(test_imu lsm6dso32)

# Installation rules
install(TARGETS lsm6dso32
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin)

install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.hpp")
