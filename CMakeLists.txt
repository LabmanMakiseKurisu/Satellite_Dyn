cmake_minimum_required(VERSION 3.14)

# 项目名称和语言
project(Satellite LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找Eign3
find_package(Eigen3 REQUIRED NO_MODULE)
# 查找Sofa
find_library(SOFA_DLL NAMES sofaDLL PATHS ${CMAKE_SOURCE_DIR}/deps/MySofa NO_DEFAULT_PATH)
if(NOT SOFA_DLL)
    message(FATAL_ERROR "sofaDLL not found")
endif()
# 查找Threads
find_package(Threads REQUIRED)

# 添加 nlohmann_json 和 cpp-httplib 作为子目录
add_subdirectory(${CMAKE_SOURCE_DIR}/deps/nlohmann_json)
add_subdirectory(${CMAKE_SOURCE_DIR}/deps/cpp-httplib)

# 添加可执行文件
file(GLOB_RECURSE SOURCES "src/*.cc") # 收集所有 src 目录下的 .cpp 文件

# 创建可执行文件目标
add_executable(${PROJECT_NAME} ${SOURCES})

# 为目标添加包含目录
target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_SOURCE_DIR}/deps # 项目的外部依赖目录
    ${CMAKE_SOURCE_DIR}/deps/MySofa # 项目的外部依赖目录
    ${CMAKE_SOURCE_DIR}/deps/cpp-httplib
    ${CMAKE_SOURCE_DIR}/deps/nlohmann_json/single_include
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/Astro
    ${CMAKE_SOURCE_DIR}/src/Componet
    ${CMAKE_SOURCE_DIR}/src/Componet/Sensors
    ${CMAKE_SOURCE_DIR}/src/Componet/Actuators
    ${CMAKE_SOURCE_DIR}/src/General
    ${CMAKE_SOURCE_DIR}/src/Satellite
    ${CMAKE_SOURCE_DIR}/src/SatelliteMath
    ${CMAKE_SOURCE_DIR}/src/util
    ${CMAKE_SOURCE_DIR}/src/api
)

# 如果找到了Eigen3库，则链接到项目
if(TARGET Eigen3::Eigen)
    target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)
endif()

# 使用完整路径链接sofaDLL
target_link_libraries(${PROJECT_NAME} PRIVATE ${SOFA_DLL}) 

# 确保线程库被链接
target_link_libraries(${PROJECT_NAME} PRIVATE ${CMAKE_THREAD_LIBS_INIT})

# 链接 nlohmann_json
target_link_libraries(${PROJECT_NAME} PRIVATE nlohmann_json::nlohmann_json)
# target_link_libraries(${PROJECT_NAME} PRIVATE cpp-httplib)




# 设置构建后的输出目录
set(RELEASE_DIR ${CMAKE_SOURCE_DIR}/build/release)

# 自定义命令：在构建完成后复制可执行文件、配置文件和动态库到 release 目录
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${RELEASE_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> ${RELEASE_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/Config ${RELEASE_DIR}/Config
    COMMAND ${CMAKE_COMMAND} -E copy ${SOFA_DLL} ${RELEASE_DIR}
)

