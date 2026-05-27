cmake_minimum_required(VERSION 3.15)
project(factor_graph)

set(CMAKE_CXX_STANDARD 17)

# 1. 寻找依赖
set(GTSAM_DIR "D:/Code/gtsam_install/CMake")
find_package(GTSAM REQUIRED CONFIG)
find_package(Boost REQUIRED COMPONENTS program_options)

# 2. 创建 DLL 库 (将 add_executable 改为 add_library)
# SHARED 表示生成 .dll 文件 (和对应的 .lib 导入库)
add_library(filter_lib SHARED
    filter.cpp
    filter.h
)

# 3. 指定头文件搜索路径
target_include_directories(filter_lib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

# 4. 链接库
target_link_libraries(filter_lib PRIVATE
    gtsam
    Boost::program_options
)

# 5. MSVC 编码设置
if(MSVC)
    target_compile_options(filter_lib PRIVATE /utf-8)
endif()