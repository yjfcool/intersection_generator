#!/bin/bash
# 快速构建脚本（无需 cmake，直接用 g++）
set -e
echo "=== 构建 intersection_generator ==="
g++ -std=c++17 -O2 \
    -Wno-unused-parameter -Wno-unused-variable -Wno-unused-but-set-variable \
    -Isrc -Ithird_party \
    src/main.cpp src/core/config.cpp \
    -o intersection_gen
echo "=== 构建成功: ./intersection_gen ==="
echo ""
echo "快速测试:"
c="./intersection_gen -c config.json -f both -o output/c test/integration/data/c_intersection.json"
t="./intersection_gen -c config.json -f both -o output/t test/integration/data/t_intersection.json"
y="./intersection_gen -c config.json -f both -o output/y test/integration/data/y_intersection.json"
r="./intersection_gen -c config.json -f both -o output/r test/integration/data/r_intersection.json"
echo "  $c" && eval $c
echo "  $t" && eval $t
echo "  $y" && eval $y
echo "  $r" && eval $r
