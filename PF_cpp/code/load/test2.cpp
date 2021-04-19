#include "mapLoader.h"
#include <iostream>


int main(void)
{
    LoadMap2 myMap;
    const char* map_str = "../../data/map/occupancy_map.txt";
    myMap.Load(map_str);

    std::cout << myMap.prob[536][635] << std::endl;
    std::cout << "rows and cols" << myMap.row << ", " << myMap.col <<std::endl;
    return 0;
}