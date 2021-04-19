#ifndef LOAD_MAP_H
#define LOAD_MAP_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>




typedef struct
{
	int resolution;					
	int size_x,size_y;				
	int min_x,min_y,max_x,max_y;	
	float offset_x, offset_y;		
	float** prob;					
} map_type;


class LoadMap
{
    public:
        LoadMap(const char* map_str);	 			
		~LoadMap();									
		int ReadFromData(const char* map_name);	    
		
		map_type* GetMap();							

    private:
		map_type* map;
   	 	
};

#endif
