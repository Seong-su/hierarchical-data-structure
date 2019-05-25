cmake_minimum_required(VERSION 3.0.0)
project(hierarchical-data-structure VERSION 0.1.0)

#Bring the headers, such as Student.h into the project
include_directories(include)

include(CTest)
enable_testing()

#Can manually add the sources using the set command as follows:
set(SOURCES src/main.cpp src/BoundingBox.cpp src/Connection.cpp src/BCTNode.cpp src/Family.cpp)

add_executable(hierarchical-data-structure ${SOURCES})

set_property(TARGET hierarchical-data-structure PROPERTY CXX_STANDARD 17)

target_link_libraries(hierarchical-data-structure -lassimp -lstdc++fs)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
