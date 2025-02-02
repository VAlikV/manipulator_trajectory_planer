#include <iostream>
#include "trajectory.hpp"
#include <time.h>

int main()
{
    Eigen::Array<double,7,1> first;
    first << 0, 0, 0, 0, 0, 0, 0;

    Eigen::Array<double,7,1> second;
    second << 5*M_PI/180,
        5*M_PI/180,
        5*M_PI/180,
        5*M_PI/180,
        5*M_PI/180,
        5*M_PI/180,
        5*M_PI/180;

    trajectory::Trajectory tr(first);

    clock_t last_time = clock();

    tr.push(second);

    std::cout << "Время: " << (((double)(clock() - last_time))/CLOCKS_PER_SEC)*1000 << std::endl;

    size_t n = tr.size();

    std::cout << "Размер: " << n << std::endl << std::endl;

    Eigen::Array<double,7,1> temp;

    for (int i = 0; i < n; ++i)
    {
        tr.pop(temp);
        std::cout << temp.transpose()*180/M_PI << std::endl;
    }

    std::cout << std::endl;
    std::cout << "Размер2: " << tr.size() << std::endl << std::endl;
    
    tr.pop(temp);

    std::cout << "Размер: " << temp.transpose()*180/M_PI << std::endl << std::endl;

}