@echo off
echo === Building intersection_generator ===
g++ -std=c++17 -O2 ^
    -Wno-unused-parameter ^
    -Isrc -Ithird_party ^
    src/main.cpp src/core/config.cpp ^
    -o intersection_gen.exe
echo === Build success: intersection_gen.exe ===
pause
