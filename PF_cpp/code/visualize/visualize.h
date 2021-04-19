#include "fssimplewindow.h"
#include "load_map.h"
#include <iostream>

class DrawMap
{
public:
    map_type *myMap;
    DrawMap()
    {
        const char* map_str = "../data/map/wean.dat";
        LoadMap loadmap(map_str);
        myMap = loadmap.GetMap();

    }
    ~DrawMap()
    {

    }

    void Draw(void) const
    {
        int wid,hei;
        FsGetWindowSize(wid,hei);
        // int xMax = myMap->max_x;
        // int yMax = myMap->max_y;
        int xMax = 800;
        int yMax = 800;
// std::cout << "xmax and ymax in draw :" << xMax <<"," <<yMax <<std::endl;
// std::cout << xMax <<std::endl;
        for(int y =0; y < yMax;++y)
        {
            for(int x = 0; x < xMax; ++x)
            {
                float c = 1 - myMap->prob[y][x];
// std::cout << c << std::endl;
                if(c == -1) c=1;
                glColor3d(c,c,c);
                glBegin(GL_POINTS);
                // glPointSize(4);
                // glVertex2i(10*x,10*y);
                glVertex2i(x,y+100);
            
            }
        }
        glEnd();
        // glColor3d(0.5,0.5,0.5);
        // glBegin(GL_LINES);
        // // glPointSize(40);
        // glVertex2d(200,200);
        // glVertex2d(300,300);
        // glEnd();

    }
};





// int main(void)
// {
//     DrawMap drawmap;

//     FsOpenWindow(0,0,1000,1000,1);
    
//     auto key = FsInkey();
//     for(;;)
//     {
//         FsPollDevice();
//         if(FSKEY_ESC==key)
//         {
// std::cout << "??" << std::endl;
//             break;
//         }

//         glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
//         drawmap.Draw();
//         FsSwapBuffers();

//         // FsSleep(25);



//     }

//     return 0;
// }