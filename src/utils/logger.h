#pragma once
#include <string>
#include <iostream>
#include <sstream>
#include <chrono>
#include <iomanip>

class Logger {
public:
    static bool verbose;

    static void info(const std::string& msg){
        if(verbose) std::cout << "[INFO]  " << msg << "\n";
    }
    static void warn(const std::string& msg){
        std::cerr << "[WARN]  " << msg << "\n";
    }
    static void error(const std::string& msg){
        std::cerr << "[ERROR] " << msg << "\n";
    }
    static void debug(const std::string& msg){
        if(verbose) std::cout << "[DEBUG] " << msg << "\n";
    }
};

inline bool Logger::verbose = false;

// 超时检测辅助
class Timer {
    std::chrono::steady_clock::time_point start_;
    int limitMs_;
public:
    Timer(int limitMs) : start_(std::chrono::steady_clock::now()), limitMs_(limitMs){}
    long elapsedMs() const {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now()-start_).count();
    }
    bool exceeded() const { return limitMs_>0 && elapsedMs()>limitMs_; }
};
