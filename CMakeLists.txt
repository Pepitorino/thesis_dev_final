cmake_minimum_required(VERSION 3.10)
project(CSE1Thesis)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# If Open3D was built manually
set(Open3D_DIR "/home/ipe/Open3D/build")

# ----------------------
# Dependencies
# ----------------------
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(octomap REQUIRED)
find_package(CGAL REQUIRED)
find_package(Open3D REQUIRED)

# ----------------------
# Include directories
# ----------------------
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${CGAL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# ----------------------
# Source files
# ----------------------
set(SOURCES
    src/main.cpp
    src/voxelstruct.cpp
    src/nbvstrategy.cpp
    src/ellipsoid.cpp
)

# ----------------------
# Build executable
# ----------------------
add_executable(CSE1Thesis ${SOURCES})

# ----------------------
# Link libraries
# ----------------------
target_link_libraries(CSE1Thesis
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${OCTOMAP_LIBRARIES}
    Open3D::Open3D
    ${CGAL_LIBRARIES}
)
