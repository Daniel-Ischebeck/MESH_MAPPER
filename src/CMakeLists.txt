


file(GLOB MY_FILES "*.tri" ${MESH_MAPPER_SOURCE_DIR}/files)
file(COPY ${MY_FILES} DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

add_library(model Point.cpp Face.cpp)

add_executable(reader TRI_reader.cpp)

target_link_libraries(reader model)