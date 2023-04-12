
list(PREPEND CMAKE_MODULE_PATH ${MESH_MAPPER_SOURCE_DIR}/cmake)

# Libigl
include(libigl)


# Enable the target igl::glfw
igl_include(glfw)

file(GLOB MY_FILES "*.tri" ${MESH_MAPPER_SOURCE_DIR}/files)
file(COPY ${MY_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_library(model Point.cpp Face.cpp Edge.cpp FileIO.cpp processing.cpp)

list(APPEND CMAKE_MODULE_PATH "C:/Users/danie/Downloads/eigen-3.4.0/eigen-3.4.0/cmake")
find_package (Eigen3 REQUIRED)

add_executable(mapper TRI_mapper.cpp)
# dense_

target_link_libraries(model Eigen3::Eigen)
target_link_libraries(mapper model)
target_link_libraries(mapper Eigen3::Eigen)
target_link_libraries(mapper igl::glfw)
