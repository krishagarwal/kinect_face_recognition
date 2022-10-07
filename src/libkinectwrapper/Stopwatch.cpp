#include "include/Stopwatch.h"
 
using namespace std;

Stopwatch::Stopwatch(GLFWwindow *win){
    window = win;
}

// start and stop times and duration
// might have to change steady_clock to system_clock for realtime clock
// but steady_clock better for calculating intervals/duration
void Stopwatch::startStopwatch() {
    start =  std::chrono::steady_clock::now();
}

void Stopwatch::endStopwatch() {
    end = std::chrono::steady_clock::now();
}

void Stopwatch::printDuration() {
    s = chrono::duration_cast<chrono::seconds>(end - start).count();
    std::cout << s << std::endl;
}