
set(CLUSTER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/quiz/cluster CACHE PATH "Cluster quiz dir")
set(RANSAC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/quiz/ransac CACHE PATH "Ransac quiz dir")
set(SENSORS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sensors CACHE PATH "Sensors dir")
set(RENDER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/render CACHE PATH "Render dir")

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} ${CLUSTER_DIR} ${RANSAC_DIR} ${SENSORS_DIR} ${RENDER_DIR})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

file(GLOB RENDER_SOURCES ${RENDER_DIR}/*.cpp)
file(GLOB RENDER_HEADERS ${RENDER_DIR}/*.h)


add_library(render ${RENDER_SOURCES} ${RENDER_HEADERS})
target_link_libraries(render ${PCL_LIBRARIES})

add_executable (environment ${CMAKE_CURRENT_SOURCE_DIR}/environment.cpp)
target_link_libraries (environment ${PCL_LIBRARIES} render)

add_executable (quizCluster ${CMAKE_CURRENT_SOURCE_DIR}/quiz/cluster/cluster.cpp)
target_link_libraries (quizCluster ${PCL_LIBRARIES} render)

add_executable (quizRansac ${CMAKE_CURRENT_SOURCE_DIR}/quiz/ransac/ransac2d.cpp)
target_link_libraries (quizRansac ${PCL_LIBRARIES} render)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/sensors/data DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/sensors/data)
