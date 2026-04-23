# Lines that begin with a # are comments
# set the minimum required version of cmake, usually the first line
cmake_minimum_required(VERSION 3.22)

# project_name sets the name of the project and causes cmake to
project(fingerlib)

# Find all external libraries.
find_package(Armadillo REQUIRED)
find_package(modern_robotics REQUIRED)
# Create a library.
add_library(fingerlib
    include/fingerlib/pos_controller.hpp
    src/pos_controller.cpp
    include/fingerlib/transformer.hpp
    src/transformer.cpp
    include/fingerlib/sinusoid.hpp
    src/sinusoid.cpp
)

# Use target_include_directories so that #include"fingerlib/header.hpp" works
target_include_directories(fingerlib
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>
    $<INSTALL_INTERFACE:include/>)

# specify additional compilation flags for the library
target_compile_options(fingerlib PUBLIC -Wall -Wextra -pedantic)

# Enable c++23 support.
target_compile_features(fingerlib PUBLIC cxx_std_23)

target_link_libraries(fingerlib
    ${ARMADILLO_LIBRARIES}
)
target_link_libraries(fingerlib
    modern_robotics::modern_robotics
)

find_package(Catch2 3 REQUIRED)

add_executable(tests tests/test_transformer.cpp)
target_link_libraries(tests PRIVATE Catch2::Catch2WithMain fingerlib)

find_package(Doxygen)

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

# build the documentation
if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
    # Turn the README.md into the homepage of the doxygen docs
    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    # Tell Doxygen where to find the documentation
    doxygen_add_docs(doxygen include/ src/ README.md ALL)

    # The documentation will be in the build/html directory
    # The main page is build/html/index.html
endif()

# install the include files by copying the whole include directory
install(DIRECTORY include/fingerlib DESTINATION include)

# Create a CMake Exported Target containing the lib and exe.
install(TARGETS fingerlib EXPORT ${PROJECT_NAME}-targets)

# The project_name-targets created by install(TARGETS) needs to be installed.
install(EXPORT ${PROJECT_NAME}-targets
        FILE ${PROJECT_NAME}-config.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION lib/cmake/${PROJECT_NAME}
)
