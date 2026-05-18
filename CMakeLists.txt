cmake_minimum_required(VERSION 3.14)
project(intersection_generator VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================
# 编译选项
# ============================================================
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -O2 -Wno-unused-parameter)
endif()
if(MSVC)
    add_compile_options(/W3 /O2 /utf-8)
endif()

# ============================================================
# 源文件收集
# ============================================================
set(SOURCES
    src/main.cpp
    src/core/config.cpp
)

# ============================================================
# 可执行文件
# ============================================================
add_executable(intersection_gen ${SOURCES})

target_include_directories(intersection_gen PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/third_party
)

# ============================================================
# 可选依赖（GEOS / proj / shapelib）
# 本工程已自实现核心功能，外部库为可选增强项
# ============================================================

# GEOS（可选：更精确的几何运算）
find_library(GEOS_C_LIB NAMES geos_c QUIET)
find_path(GEOS_INCLUDE NAMES geos_c.h QUIET)
if(GEOS_C_LIB AND GEOS_INCLUDE)
    target_include_directories(intersection_gen PRIVATE ${GEOS_INCLUDE})
    target_link_libraries(intersection_gen PRIVATE ${GEOS_C_LIB})
    target_compile_definitions(intersection_gen PRIVATE HAVE_GEOS)
    message(STATUS "GEOS found: ${GEOS_C_LIB}")
else()
    message(STATUS "GEOS not found - using built-in geometry routines")
endif()

# proj（可选：精确坐标变换）
find_library(PROJ_LIB NAMES proj QUIET)
find_path(PROJ_INCLUDE NAMES proj.h QUIET)
if(PROJ_LIB AND PROJ_INCLUDE)
    target_include_directories(intersection_gen PRIVATE ${PROJ_INCLUDE})
    target_link_libraries(intersection_gen PRIVATE ${PROJ_LIB})
    target_compile_definitions(intersection_gen PRIVATE HAVE_PROJ)
    message(STATUS "proj found: ${PROJ_LIB}")
else()
    message(STATUS "proj not found - using built-in AEQD projection")
endif()

# ============================================================
# 安装
# ============================================================
install(TARGETS intersection_gen DESTINATION bin)
install(FILES config.json DESTINATION share/intersection_gen)
install(DIRECTORY test/integration/data
        DESTINATION share/intersection_gen/test_data)

# ============================================================
# 测试
# ============================================================
option(BUILD_TESTS "Build tests" ON)
if(BUILD_TESTS)
    enable_testing()

    # 集成测试：十字路口
    add_test(NAME test_cross
        COMMAND intersection_gen
            -c ${CMAKE_SOURCE_DIR}/config.json
            -f both
            -o ${CMAKE_BINARY_DIR}/test_output/cross
            ${CMAKE_SOURCE_DIR}/test/integration/data/cross_intersection.json
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )

    # 集成测试：T字路口
    add_test(NAME test_t
        COMMAND intersection_gen
            -c ${CMAKE_SOURCE_DIR}/config.json
            -f both
            -o ${CMAKE_BINARY_DIR}/test_output/t
            ${CMAKE_SOURCE_DIR}/test/integration/data/t_intersection.json
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )

    # 集成测试：环岛
    add_test(NAME test_roundabout
        COMMAND intersection_gen
            -c ${CMAKE_SOURCE_DIR}/config.json
            -f both
            -o ${CMAKE_BINARY_DIR}/test_output/roundabout
            ${CMAKE_SOURCE_DIR}/test/integration/data/roundabout.json
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    )

    set_tests_properties(test_cross test_t test_roundabout
        PROPERTIES TIMEOUT 30
    )
endif()
