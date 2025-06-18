cmake_minimum_required(VERSION 3.14)  # 3.14+ recommandé pour FetchContent

project(ft_kalman VERSION 1.0 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -g")

# FetchContent pour importer Eigen automatiquement
include(FetchContent)
FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG        3.4.0  # ou la version souhaitée
)
FetchContent_MakeAvailable(eigen)

# Source files and headers
set(SRC_FILES
    src/main.cpp
    src/utils.cpp
    src/UdpClient.cpp
    src/DataStorage.cpp
    src/KalmanFilter.cpp
)

set(HEADER_FILES
    inc/main.h
    inc/UdpClient.h
    inc/KalmanFilter.h
    inc/DataStorage.h
)

# Output executable
add_executable(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})

# Inclure Eigen dans l'include path
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

# Custom clean target
add_custom_target(clean_repo
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}/obj
    COMMAND ${CMAKE_COMMAND} -E remove ${PROJECT_NAME}
)

# Custom rebuild target
add_custom_target(rebuild
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target clean
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR}
)
