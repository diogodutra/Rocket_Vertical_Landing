cmake_minimum_required(VERSION 3.10)
project(RocketGNC_Embedded)
enable_testing()

# Set Compiler Standards (Ensuring Modern/Safety-Critical C++)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Define the GNC Library
add_library(gnc_lib src/GNCController.cpp)

# Include Directories
target_include_directories(gnc_lib PUBLIC include)

# Define the Test Runner Executable
add_executable(run_gnc_tests tests/test_gnc_embedded.cpp)

# Link the Test Runner with our GNC Library
target_link_libraries(run_gnc_tests PRIVATE gnc_lib)
target_include_directories(run_gnc_tests PRIVATE include)

# Add standard compiler flags for safety
if(MSVC)
    target_compile_options(gnc_lib PRIVATE /W4 /WX) # treat warnings as errors on Windows
else()
    target_compile_options(gnc_lib PRIVATE -Wall -Wextra -Werror) # treat warnings as errors on Linux/Mac
endif()

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)
# For Windows: prevent gtest from overriding our compiler settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Link against our GNC library and GoogleTest
target_link_libraries(run_gnc_tests PRIVATE gnc_lib GTest::gtest_main)

# Enable the 'ctest' command
include(GoogleTest)
gtest_discover_tests(run_gnc_tests)