#include <iostream>
#include <string>
#include <chrono>
#include <ros/ros.h>

class TicToc
{
    std::string name;
    std::chrono::time_point<std::chrono::high_resolution_clock> start;
    bool toc_done;

public:
    TicToc(std::string _name)
    :name(_name)
    {
        start = std::chrono::high_resolution_clock::now();
        toc_done = false;
    }

    void Toc()
    {
        if(toc_done == false)
        {
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            ROS_INFO("TIME for \"%s\": %.3fms", name.c_str(), elapsed.count()*1000);
            toc_done = true;
        }
    }

    ~TicToc()
    {
        Toc();
    }
};
