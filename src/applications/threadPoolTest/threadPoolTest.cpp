#include "ThreadPool.h"
#include <vector>
#include <functional>
#include <iostream>

int n = 10;

int result[10] = {0};

class Work {
    int num;
public:
    Work(int x) : num(x){}

    void work(char dummy){
        result[num]++;
    }
};


int main(int argc, char **argv) {
    ThreadPoolQueue pool{n};
    std::cout << "starting" << std::endl;

    std::vector<Work*> todo;
    std::vector<std::packaged_task<void()>> tasks;
    std::vector<std::future<void>> done;
    // Generate work
    for(int i = 0; i < n * 100; i++){
        todo.push_back(new Work(i % n));

        // Create std::packaged_task
        Work* cur = todo.back();
        auto job = std::bind(&Work::work, cur, 'd');
        std::packaged_task<void()> task(job);

        // Submit job, keep task around
        auto future = pool.addJob(std::move(task));
        done.push_back(std::move(future));
    }

    std::cout << "submitted" << std::endl;

    // Use future for later fine-grain barrier
    for(int i = 0; i < n; i++){
        done[i].get();
    }

    for(int i = 0; i < n; i++){
        std::cout << result[i] << " ";
    }
    std::cout << "\n";

    std::cout << "exiting" << std::endl;
    return 0;
}