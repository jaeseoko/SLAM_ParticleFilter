#include "load_map.h"

#include <iostream>

LoadMap::LoadMap(const char* map_str)
{
    this->map = (map_type*) malloc(sizeof(map_type));    
    ReadFromData(map_str);
}

LoadMap::~LoadMap()
{

}

int LoadMap::ReadFromData(const char* map_name)
{
    int x, y, count;
    float temp;
    char line[256];
    FILE *fp;

    if((fp = fopen(map_name, "rt")) == NULL)     
    {
        fprintf(stderr, "ERROR: Could not open file %s\n", map_name);
        return -1;
    }
    fprintf(stderr, "# Reading map: %s\n", map_name);

    
    while((fgets(line, 256, fp) != NULL)
          && (strncmp("global_map[0]", line , 13) != 0))
    {
        if(strncmp(line, "robot_specifications->resolution", 32) == 0)
    
            if(sscanf(&line[32], "%d", &(map->resolution)) != 0)
                printf("# Map resolution: %d cm\n", map->resolution);
        if(strncmp(line, "robot_specifications->autoshifted_x", 35) == 0)
            if(sscanf(&line[35], "%g", &(map->offset_x)) != 0) 
            {
                map->offset_x = map->offset_x;
                printf("# Map offsetX: %g cm\n", map->offset_x);
            }
        if(strncmp(line, "robot_specifications->autoshifted_y", 35) == 0) 
        {
            if (sscanf(&line[35], "%g", &(map->offset_y)) != 0) 
            {
                map->offset_y = map->offset_y;
                printf("# Map offsetY: %g cm\n", map->offset_y);
            }
        }
    }

    if(sscanf(line,"global_map[0]: %d %d", &map->size_y, &map->size_x) != 2)  
    {
        fprintf(stderr, "ERROR: corrupted file %s\n", map_name);
        fclose(fp);
        return -1;
    }
    printf("# Map size: %d %d\n", map->size_x, map->size_y);


    map->prob = (float **)calloc(map->size_x, sizeof(float *));   
    for(int i = 0; i < map->size_x; i++)
    {
        map->prob[i] = (float *)calloc(map->size_y, sizeof(float));
    }
    
    map->min_x = map->size_x;
    map->max_x = 0;
    map->min_y = map->size_y;
    map->max_y = 0;
    count = 0;
    for(x = 0; x < map->size_x; x++)
    {
        for(y = 0; y < map->size_y; y++, count++)
        {
            if(count % 10000 == 0)
            {
                fprintf(stderr, "\r# Reading ... (%.2f%%)", count / (float)(map->size_x * map->size_y) * 100);
            }
            fscanf(fp,"%e", &temp);         

            if(temp < 0.0)
            {
                map->prob[x][y] = -1;       
            }
            else
            {
                if(x < map->min_x)          
                    map->min_x = x;
                else if(x > map->max_x)
                    map->max_x = x;
                if(y < map->min_y)
                    map->min_y = y;
                else if(y > map->max_y)
                    map->max_y = y;
                map->prob[x][y] = 1 - temp;
// std::cout<< "check assigned prob from map: "<<map->prob[x][y] <<std::endl;  
            }
        }
    }
    
    fprintf(stderr, "\r# Reading ... (%.2f%%)\n\n",count / (float)(map->size_x * map->size_y) * 100);
    fclose(fp);
    return 0;
}




map_type* LoadMap::GetMap()
{
    return this->map;
}
