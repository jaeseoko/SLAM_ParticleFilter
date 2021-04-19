#include "visualize.h"
#include "fssimplewindow.h"



int main(void)
{
    DrawMap drawing;

    FsOpenWindow(0,0,1000,1000,1);
    
    auto key = FsInkey();
    for(;;)
    {
        FsPollDevice();
        if(FSKEY_ESC==key)
        {
            break;
        }

        glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT);
        drawing.Draw();
        FsSwapBuffers();

        FsSleep(25);



    }

    return 0;
}