#ifndef STOPWATCH_H
#define STOPWATCH_H
#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <GLFW/glfw3.h>
#include "MultiKinectPacket.h"

class Stopwatch{
private:
    GLFWwindow *window;
    std::chrono::steady_clock::time_point start;
    std::chrono::steady_clock::time_point end;
    std::chrono::duration<int64_t, std::ratio<1L, 1L>>::rep s;
public: 
    Stopwatch(GLFWwindow *window);
    void startStopwatch();
    void endStopwatch();
    void printDuration();

};

#endif