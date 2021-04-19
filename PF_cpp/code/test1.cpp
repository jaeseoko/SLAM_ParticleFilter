#include "load_map.h"
#include <iostream>



int main(void)
{
    const char* map_str = "../data/map/wean.dat";
    LoadMap loadmap(map_str);
    auto myMap = loadmap.GetMap();
    std::cout << myMap->prob[430][450] << std::endl;
}