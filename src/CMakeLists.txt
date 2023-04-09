
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

add_executable(reader dense_TRI_mapper.cpp)


target_link_libraries(model Eigen3::Eigen)
target_link_libraries(reader model)
target_link_libraries(reader Eigen3::Eigen)
target_link_libraries(reader igl::glfw)
# target_link_libraries (reader
#                         model
#                         ${Eigen_LIBRARIES}
# )