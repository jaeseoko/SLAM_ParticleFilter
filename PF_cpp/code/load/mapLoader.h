#ifndef MAPLOADER_H
#define MAPLOADER_H
#include <iostream>
#include <fstream>
#include <string>

class LoadMap2
{
public:
    int row = 0;
    int col = 0;
    double prob[800][800];

    void Load(const char* map_str)
    {
std::cout << __FUNCTION__ <<__LINE__<<std::endl;
        std::ifstream file(map_str);
        std::string str;
        while (std::getline(file, str)) 
        {
            std::string hash = "#\n";

            if (str.length() <= 2) {
                row += 1;
                col = 0;
                continue;
            }
            //std::cout << "str is " << str << std::endl;
            prob[row][col] = std::stod(str);
            col++;
            //std::cout << str << "\n";
        }
        row = 800;
        col = 800;
        std::cout << "done loading array" << std::endl;
        
    }
};

#endif